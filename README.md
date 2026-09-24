# Qb2 ROS2 Driver

ROS2 driver for the Qb2 LiDAR devices of Blickfeld GmbH.

This fork builds on **ROS 2 Lyrical / Ubuntu 26.04**. Authenticated live streaming and RViz have been verified with a physical Qb2.
The revised driver has also passed live streaming, configuration reads, dry runs,
and applied frame-rate/range-filter changes with readback on **AMINA firmware
3.0.9**. The original scan settings and filters were restored after testing.

## Build on Lyrical

From your colcon workspace, with this repository under `src/`:

```bash
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src/blickfeld_qb2_ros2_driver --ignore-src -r -y
colcon build --symlink-install --packages-select blickfeld_qb2_ros2_driver \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

The native prerequisites include `libboost-dev`, `libgrpc++-dev`,
`protobuf-compiler-grpc`, `libprotobuf-dev`, and ROS's `diagnostic_updater` and
`rclcpp_components`. The first CMake configure downloads the public
[Blickfeld Qb2 SDK](https://github.com/Blickfeld/blickfeld-qb2), pinned to
v3.0.7 (`181c2e06991155cec5b3a490a179ac6c9673eb9f`). To select a protocol
revision for another firmware, pass `-DBLICKFELD_QB2_REVISION=<commit-or-tag>`
through `--cmake-args` and rebuild. Only the default pin was tested here.
An offline SDK checkout can instead be supplied with
`-DFETCHCONTENT_SOURCE_DIR_BLICKFELD-QB2=/absolute/path/to/blickfeld-qb2`.

The build uses Boost headers rather than the removed `Boost::system` library
and the ROS-provided GoogleTest package rather than downloading a second copy.
`BUILD_TESTING` is the test switch. SDK-generated sources may produce unused
protobuf-import and deprecated-API warnings.

## Live streaming

Replace `192.168.1.100` with your sensor's hostname or IP:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch blickfeld_qb2_ros2_driver blickfeld_qb2_ros2_driver.launch.py \
  fqdn:=192.168.1.100
```

Launch defaults publish `/bf/points_raw` in frame `lidar`, including `intensity`
and `point_id` as UINT32 fields. These defaults are set by the launch file;
direct component loading retains the upstream node defaults. Available options:

```bash
ros2 launch blickfeld_qb2_ros2_driver blickfeld_qb2_ros2_driver.launch.py --show-args
```

For authenticated devices, put the application key in a private file and supply
`serial_number:=<device-serial>` and `application_key_file:=/absolute/path/to/key`.
The file is read at startup; its contents are not stored in a ROS parameter.
The legacy `application_key` parameter remains supported, but its value is
visible through ROS parameter services. Do not supply both forms.
The key requires the matching serial number; empty credentials are the default.
The SDK uses TCP 55551 for TLS and TCP 50051 to discover the serial number when
one is not supplied. See [sensor configuration](doc/sensor_configuration.md) for
the scan-pattern and point-cloud-filter services. Startup does not modify sensor settings.
Keep `use_measurement_timestamp:=false` until sensor and host clocks are
synchronized. The driver does not publish a TF transform; in RViz, set the
fixed frame to `lidar` and add a PointCloud2 display for `/bf/points_raw`.

In another sourced terminal, once the sensor is connected:

```bash
ros2 topic hz /bf/points_raw
ros2 topic echo /bf/points_raw --once --field header
ros2 topic echo /diagnostics --once
```

## Selectable scan profiles

Switch profiles while the driver and RViz keep running:

```bash
ros2 run blickfeld_qb2_ros2_driver qb2_profile --list
ros2 run blickfeld_qb2_ros2_driver qb2_profile balanced --apply  # 4 Hz
ros2 run blickfeld_qb2_ros2_driver qb2_profile motion --apply    # 6 Hz
ros2 run blickfeld_qb2_ros2_driver qb2_profile mapping --apply   # 2 Hz
```

Omit `--apply` to validate without changing the sensor. The profiles change only
the target frame rate; the sensor adjusts scanline counts during uniform scanning.
On the tested Qb2, mapping produced denser frames and motion produced faster updates.
Device limits are checked for every request. These rates are starting points for
the tested 90° × 50° uniform scan, not universal presets for every configuration.

Use the same `ROS_DOMAIN_ID` and discovery settings as the running driver. For
the local test session, set `ROS_DOMAIN_ID=170` and
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. A namespaced driver needs, for example,
`--driver-node /front/blickfeld_qb2_driver`. See
[profile customization and measured results](doc/sensor_configuration.md#selectable-profiles).

## ROS interfaces and composition

Both components also have standalone executables:

```bash
ros2 run blickfeld_qb2_ros2_driver qb2_driver --ros-args -p fqdn:=192.168.1.100
ros2 run blickfeld_qb2_ros2_driver qb2_snapshot_driver --ros-args --params-file /absolute/path/to/snapshot.yaml
```

Driver settings are read-only after startup; changing a connection, output field,
frame, or snapshot schedule requires restarting the component. Sensor settings
use explicit services with validation and readback, rather than parameters that
claim to change without updating the device. Caller-provided `NodeOptions`,
including intra-process communication, are respected.

Both launch files accept `namespace:=front`. Default relative topic names then
resolve to `/front/bf/points_raw`; an explicitly absolute topic remains absolute.
The driver retains reliable, volatile, depth-1 publisher defaults for existing
subscribers. ROS's standard QoS overrides allow a sensor-data profile, for example:

```bash
ros2 run blickfeld_qb2_ros2_driver qb2_driver --ros-args \
  -p fqdn:=192.168.1.100 -p point_cloud_topic:=points \
  -p qos_overrides./points.publisher.reliability:=best_effort \
  -p qos_overrides./points.publisher.depth:=5
```

Use the fully resolved topic name in QoS override keys. Overrides are startup
settings. The same parameters can be supplied in a component's YAML configuration.

The point coordinates remain in the vendor's sensor coordinate system, in metres.
Changing `frame_id` only changes the label; it does not rotate points. See the
[vendor coordinate diagram](https://docs.blickfeld.com/qb2/Qb2/working_principles/coordinate_system.html)
and supply the measured mounting transform in TF when integrating with a robot.
[REP 103](https://www.ros.org/reps/rep-0103.html) uses x-forward/y-left/z-up body
frames; this driver preserves its existing raw-coordinate convention for compatibility.

## Snapshot mode

Copy [config/snapshot.yaml](config/snapshot.yaml), enter the devices, and keep
all five `fqdn_*`/`fqdns` lists the same length. Use an empty string for each
unused serial number or application key. Alternatively, supply
`fqdn_application_key_files` with one path (or empty string) per device; the raw
key list may then be omitted. Do not give both a key and key file for the same
device. Then launch your copy:

```bash
ros2 launch blickfeld_qb2_ros2_driver blickfeld_qb2_ros2_snapshot_driver.launch.py \
  params_file:=/absolute/path/to/snapshot.yaml
ros2 service call /bf/trigger_snapshot std_srvs/srv/Trigger '{}'
```

`snapshot_frame_rate: 0.0` enables manual triggering only, including at startup;
`0.1` captures immediately and every ten seconds thereafter. Invalid device-list
lengths, rates, or credentials reject component loading without shutting down
the whole container.

## Verification

```bash
colcon test --packages-select blickfeld_qb2_ros2_driver --event-handlers console_direct+
colcon test-result --test-result-base build/blickfeld_qb2_ros2_driver --verbose
```

On Lyrical, 31 GoogleTest cases cover cloud fields/timestamps, malformed frames,
startup validation, credential files, diagnostics, frame counters, component
unloading, and configuration requests against a local gRPC mock sensor. The
mock tests exercise cancellation before/after context creation, partial updates,
device-limit checks, dry runs, authentication failures, rejected writes, and
failed readback. Nine Python tests cover profile loading, invalid rates/fields,
and explicit apply versus default dry-run requests.

Both installed launch files were checked with namespaces, publisher and service
discovery, and clean SIGINT shutdown. A separate ROS service-client check verified
standalone startup, read-only parameters, best-effort/depth-5 QoS overrides,
and explanatory errors for an offline device.

See [the improvement review](doc/driver_review.md) for benchmark results and
remaining integration work.

Please check the [Antora documentation](doc/modules/ROOT/pages/index.adoc) for more info.

The "Getting Started" guides can be found at [Qb2 Documentation](https://docs.blickfeld.com/qb2/).
