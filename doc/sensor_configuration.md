# Sensor configuration through ROS 2

The live driver provides these private services (prepend its namespace, if any):

| Service | ROS service type |
| --- | --- |
| `/blickfeld_qb2_driver/get_scan_pattern` | `blickfeld_qb2_ros2_driver/srv/GetScanPattern` |
| `/blickfeld_qb2_driver/set_scan_pattern` | `blickfeld_qb2_ros2_driver/srv/SetScanPattern` |
| `/blickfeld_qb2_driver/get_point_cloud_filter` | `blickfeld_qb2_ros2_driver/srv/GetPointCloudFilter` |
| `/blickfeld_qb2_driver/set_point_cloud_filter` | `blickfeld_qb2_ros2_driver/srv/SetPointCloudFilter` |

Snapshot mode exposes the same suffixes under
`/blickfeld_qb2_snapshot_driver/devices/device_0`, `device_1`, etc. Indices follow
FQDN devices first, then Unix-socket devices, in the configured order.

## Selectable profiles

The installed `qb2_profile` client selects one of three local profiles through
the running driver's `set_scan_pattern` service. It updates only the enabled
target frame rate, validates against current device limits, and reads back applied
changes. FoV, pulse mode, frame mode, and point-cloud filters are preserved.
It does not enable uniform scanning; when uniform scanning is already enabled,
the sensor adjusts scanline counts to reach the requested rate.

Measured on a Qb2 running AMINA 3.0.9, at 90° × 50° FoV, uniform scanning,
up-only frame mode, and pulse mode C:

| Profile | Target | Measured rate | Median points/frame | Up scanlines |
| --- | ---: | ---: | ---: | ---: |
| `balanced` | 4 Hz | 4.002 Hz | 7,360 | 100 |
| `motion` | 6 Hz | 6.005 Hz | 4,119 | 56 |
| `mapping` | 2 Hz | 2.001 Hz | 17,427 | 236 |

These are short observations of one scene, not guaranteed point counts or a
mapping-quality benchmark. Device-reported limits allowed up to about 7.98 Hz
with this scan configuration. Other FoVs, modes, firmware, or models can produce
different limits and density tradeoffs.

```bash
source install/setup.bash
# Match these to your running driver (the current local RViz session uses 170):
export ROS_DOMAIN_ID=170 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

ros2 run blickfeld_qb2_ros2_driver qb2_profile --list
ros2 run blickfeld_qb2_ros2_driver qb2_profile mapping            # validate only
ros2 run blickfeld_qb2_ros2_driver qb2_profile mapping --apply    # denser frames, 2 Hz
ros2 run blickfeld_qb2_ros2_driver qb2_profile motion --apply     # faster updates, 6 Hz
ros2 run blickfeld_qb2_ros2_driver qb2_profile balanced --apply   # restore 4 Hz
```

RViz does not need to restart. The command prints JSON containing `success`,
`applied`, and the sensor configuration, and exits nonzero on failure. The default
is a dry run; `--dry-run` makes that explicit. Readback after an apply includes the
accepted actual frame rate and adjusted scanline counts.

To customize rates, copy [config/profiles.yaml](../config/profiles.yaml), edit its
`target_frame_rate` values, and pass `--profiles-file /path/to/profiles.yaml`.
The file must define all three profiles with finite positive rates. For a
namespaced driver, add `--driver-node /front/blickfeld_qb2_driver`; for a snapshot
device, use its device prefix from the service table above. These are local
presets, not stored sensor presets. Applying a custom scan clears the active
named-preset label; stored patterns are not overwritten or deleted. The driver
does not reapply profiles on startup.

## Direct configuration services

Read current settings:

```bash
ros2 service call /blickfeld_qb2_driver/get_scan_pattern \
  blickfeld_qb2_ros2_driver/srv/GetScanPattern '{}'
ros2 service call /blickfeld_qb2_driver/get_point_cloud_filter \
  blickfeld_qb2_ros2_driver/srv/GetPointCloudFilter '{}'
```

Set requests have an explicit `fields` list, a typed `configuration`, and
`dry_run` (default **true**). Only listed fields are replaced in a freshly fetched
configuration. Unlisted settings, including vendor fields not exposed in ROS,
are preserved. Unknown, duplicate, empty, and read-only field selections fail.

| Scan field selector | Meaning |
| --- | --- |
| `horizontal_field_of_view` | Horizontal optical FoV, radians |
| `vertical_field_of_view` | Vertical FoV, radians |
| `scanlines_up`, `scanlines_down` | Positive scanline counts |
| `angle_spacing` | Horizontal sample spacing, radians; unavailable while uniform scanning controls it |
| `frame_mode` | `1`: up/down combined; `2`: up only; `3`: separate up/down frames |
| `target_frame_rate` | Updates `target_frame_rate_enabled` and `target_frame_rate` (Hz) together |
| `foveation` | Updates `foveation_enabled`, `foveation_fraction`, and `foveation_density_factor` together |

`actual_frame_rate`, `uniform`, and `pulse_mode` are observations, not writable
fields. A dry-run scan preview has no predicted actual frame rate. The driver
queries the device's `GetLimits` for the proposed scan pattern and checks the
reported coupled limits before `Set`. Unsupported firmware/API errors are
returned; writes are not attempted if limit retrieval fails. Firmware still
performs final validation on apply.

For example, validate a denser scan without changing the device:

```bash
ros2 service call /blickfeld_qb2_driver/set_scan_pattern \
  blickfeld_qb2_ros2_driver/srv/SetScanPattern \
  '{fields: [target_frame_rate], configuration: {target_frame_rate_enabled: true, target_frame_rate: 2.0}, dry_run: true}'
```

| Filter field selector | Meaning |
| --- | --- |
| `minimum_range`, `maximum_range` | Metres, 0–100; zero selects firmware defaults (1 m / 100 m) |
| `minimum_reflectivity` | Vendor reflectivity threshold, 0–10000 |
| `maximum_returns_per_point` | Maximum echoes per direction; zero uses firmware default |
| `sorting` | `0`: default; `1`: intensity; `2`: reflectivity; `3`: range |
| `reflectivity_range` | Updates `reflectivity_range_enabled`, `reflectivity_range_minimum`, and `reflectivity_range_maximum` together; metres, 0–250 |

```bash
ros2 service call /blickfeld_qb2_driver/set_point_cloud_filter \
  blickfeld_qb2_ros2_driver/srv/SetPointCloudFilter \
  '{fields: [minimum_range, maximum_range], configuration: {minimum_range: 2.0, maximum_range: 40.0}, dry_run: true}'
```

Filter dry runs perform local validation; the API has no separate filter-validation
RPC. Change `dry_run` to `false` to apply either request. Successful writes are
followed by a fresh read, so the response reflects the device's accepted settings.
These settings affect the sensor and all its consumers, including the web viewer.
No sensor writes occur merely from launching the driver, loading a YAML file,
or calling a get service. The driver does not call scan-pattern Store/Delete.

Always inspect `success`, `message`, and `applied` in set responses. In particular,
`success: false, applied: true` means the write succeeded but readback failed;
read the settings again before retrying a write. An RPC timeout can have an
uncertain device-side outcome even with `applied: false`; verify before retrying.
Authentication and firmware errors are returned in `message` with the gRPC code.
Configuration requests have a five-second RPC budget; initial SDK connection can
take up to its ten-second connection timeout. Calls from this driver are serialized
per sensor, but another client/web page can still modify the device concurrently.

The SDK protocol is pinned to v3.0.7. Supported values and available controls vary
with firmware and model. On a physical Qb2 running AMINA 3.0.9, reads, dry runs,
and applied 2/4/6 Hz scan changes passed with healthy streaming diagnostics.
A temporary 5 m maximum-range filter bounded observed points to 5 m within float
precision. The original 4 Hz scan configuration and all filter settings were
restored and verified by readback. The active scan is now custom (unnamed), with
the original settings. Other writable controls and failed readbacks have mock
coverage but have not been exercised by writing them to this physical sensor.

Sources: [Blickfeld scan-pattern guide](https://docs.blickfeld.com/qb2/Qb2/guides/system_scan_pattern.html),
[pinned scan-pattern protocol](https://github.com/Blickfeld/blickfeld-qb2/blob/181c2e06991155cec5b3a490a179ac6c9673eb9f/protocol/blickfeld/system/config/scan_pattern.proto),
[pinned filter protocol](https://github.com/Blickfeld/blickfeld-qb2/blob/181c2e06991155cec5b3a490a179ac6c9673eb9f/protocol/blickfeld/core_processing/config/point_cloud.proto).
