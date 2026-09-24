# Qb2 ROS2 Driver

ROS2 driver for the Qb2 LiDAR devices of Blickfeld GmbH.

This fork builds on **ROS 2 Lyrical / Ubuntu 26.04**. The live and snapshot
components have been tested without a connected sensor; physical-device
streaming and authentication still need verification.

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

If application-key authentication is enabled on the device, also supply
`serial_number:=<device-serial>` and `application_key:=<application-key>`.
The key requires the matching serial number; empty credentials are the default.
The SDK uses TCP 55551 for TLS and TCP 50051 to discover the serial number when
one is not supplied. Configure the sensor's scan pattern in its web interface.
Keep `use_measurement_timestamp:=false` until sensor and host clocks are
synchronized. The driver does not publish a TF transform; in RViz, set the
fixed frame to `lidar` and add a PointCloud2 display for `/bf/points_raw`.

In another sourced terminal, once the sensor is connected:

```bash
ros2 topic hz /bf/points_raw
ros2 topic echo /bf/points_raw --once --field header
ros2 topic echo /diagnostics --once
```

## Snapshot mode

Copy [config/snapshot.yaml](config/snapshot.yaml), enter the devices, and keep
all five `fqdn_*`/`fqdns` lists the same length. Use an empty string for each
unused serial number or application key. Then launch your copy:

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

On Lyrical, 15 GoogleTest cases cover all intensity/point-ID combinations,
coordinates, timestamps, empty/truncated frames, startup validation, manual
snapshot loading, and component unloading while ROS remains active. The tests
use localhost for unavailable-device checks and need no sensor. Both installed
launch files were also checked for component loading, point-cloud publishers,
diagnostics, the snapshot trigger service, and clean SIGINT shutdown. No physical
Qb2 data was received during these checks.

Please check the [Antora documentation](doc/modules/ROOT/pages/index.adoc) for more info.

The "Getting Started" guides can be found at [Qb2 Documentation](https://docs.blickfeld.com/qb2/).
