# Driver improvement review (Lyrical, 2026-09-24)

## Implemented

- Protected gRPC context replacement/reset against concurrent cancellation. Permanent
  shutdown also cancels contexts created after a connection attempt completes.
  Readers are destroyed before their contexts. Snapshot retries stop on unload.
- Protected shared diagnostic state and moved elapsed-time checks to a steady clock.
  Frequency bounds are refreshed inside the diagnostic callback, avoiding shared
  `double` pointer races. Manual snapshots do not produce periodic-rate alarms;
  sparse snapshots check timestamps only when new messages were published.
- Prefer scan-pattern `actual` frame rate, with a legacy `maximum` fallback.
  Preserve authentication failures and classify gRPC PERMISSION_DENIED and
  UNAUTHENTICATED by status code. Correct frame-zero gap accounting and count
  malformed frames as dropped.
- Startup settings now declare read-only descriptors. Respect caller NodeOptions.
  Both launch files support namespaces and relative topic/service defaults.
  Standard ROS QoS overrides are supported while preserving reliable/depth-1 defaults.
- Added standalone executables alongside the existing composable nodes, and replaced
  repeated legacy dependency variables with imported CMake targets.
- Added private key-file inputs for live and snapshot authentication, so key contents
  need not be available through ROS parameter services.
- Added typed, partial-update scan-pattern and point-cloud-filter services with dry
  runs, device-limit queries, preserved vendor fields, and readback. See
  [service usage and limitations](sensor_configuration.md).
- Added an installed `qb2_profile` ROS client with selectable balanced (4 Hz),
  motion (6 Hz), and mapping (2 Hz) profiles, configurable through YAML. It defaults
  to dry-run validation and requires `--apply` to change the running sensor.
- Bulk-copy XYZ-only clouds and copy packed XYZ/IDs directly for enriched clouds;
  widen photon counts bytewise while preserving little-endian wire format.
  Avoid field-vector lookups in the point loop and avoid a snapshot protobuf copy.

## Conversion benchmark

Microseconds per conversion; smaller is better:

| Points | Fields | Before | After | Speedup |
| ---: | --- | ---: | ---: | ---: |
| 4,000 | xyz | 25.65 | 0.84 | 30.5× |
| 4,000 | xyz_intensity_id | 51.61 | 9.12 | 5.7× |
| 100,000 | xyz | 643.70 | 25.04 | 25.7× |
| 100,000 | xyz_intensity_id | 1308.81 | 233.78 | 5.6× |

Measured on this host against the previous `419c8c6` installed library and the
revised Release library. Both use `-O3 -DNDEBUG -std=gnu++20`. Each measurement is
the median of seven 200-conversion batches; two runs per version were interleaved
before/after/after/before on one pinned CPU. Checksums matched and field-layout
regression tests passed. The standalone benchmark source is in
`tests/benchmark_point_cloud.cpp`; build its explicit `benchmark_point_cloud` target.
This measures conversion only, excluding gRPC, DDS, sensor scanning, and ROS
subscribers. It does not imply a higher hardware frame rate; the live Qb2 was
previously measured at approximately 4.05 Hz.

## Validation

31 GoogleTest cases passed (35 entries in colcon including CTest wrappers).
Nine additional Python tests passed for profile loading, invalid rates/fields,
and explicit apply versus default dry-run requests.
Mock gRPC tests cover partial configuration, dry runs, device bounds, auth failures,
rejected writes, failed readback, and cancellation across context creation.
Both namespaced launch files and the standalone executable passed ROS discovery
and shutdown checks. A ROS client verified standard QoS overrides, rejected
startup-parameter changes, and typed service errors for an offline sensor.

The revision was first built separately in `build/qb2-review` and `install/qb2-review`.
After successful hardware validation, it was installed into the normal workspace
`build`/`install` tree, available through `source install/setup.bash`.
The revised driver was then verified against physical Qb2 `TDTQCAAAF` at
`10.1.10.104`, which reports **AMINA v3.0.9** (revision `0e9b7b4d`) and an idle
firmware installer. The authenticated test received **62 clouds** containing
5,965–6,007 points each; all layouts and XYZ coordinates were valid. The last
five-second diagnostic window measured **4.000002 Hz** against a 4 Hz target,
with all diagnostic statuses OK, zero dropped frames, and zero detected reboots.
The whole subscriber observation, including startup, measured approximately 4.00 Hz.

Scan-pattern and filter reads succeeded, as did a scan-pattern dry run using the
device's current limits and a locally validated filter dry run. Before/after
configuration comparisons matched exactly; that initial check performed no writes.
The application key remained absent from the ROS parameter value;
only its file path was exposed. The launch supervisor and component container both exited cleanly and the temporary
browser route was left active. Raw local evidence is in
`/tmp/qb2-firmware309-live-check.json` and `/tmp/qb2-firmware309-driver.log`.
The protocol pin remains SDK v3.0.7;
SDK and firmware version numbers are not assumed to be interchangeable.

Subsequent tuning exercised actual scan and filter writes while RViz remained
open. Mapping (2 Hz), motion (6 Hz), and balanced (4 Hz) all achieved their target
rates with healthy diagnostics. Median points per frame in this scene were
17,427, 4,119, and 7,360 respectively. A temporary 5 m maximum range bounded
observed points to 5 m within float precision. Original scan settings and filters
were restored exactly by ROS configuration readback; the active named-preset
label changed from `Dashboard` to custom/unnamed. Stored patterns were untouched.
Raw evidence is in `/tmp/qb2-live-tuning-results.json`, with details in
[selectable profiles](sensor_configuration.md#selectable-profiles).
The installed CLI also validated all three profiles against the sensor and
applied mapping followed by balanced with successful readback.

## Remaining integration work

- Physical tests cover frame-rate changes and maximum-range filtering. Other
  exposed controls, device readback failures, and longer tuning runs remain to
  be exercised on hardware.
- Coordinates retain the vendor convention. A robot integration needs its measured
  TF mounting transform; no guessed `base_link`/`map` transform is published.
  An explicit coordinate-conversion option and per-point time/return metadata
  would help downstream motion compensation and SLAM.
- Device health/temperature and acceleration streams are additional useful ROS
  interfaces. Percept/background subtraction belongs to the separate processing
  API and may depend on firmware/product licensing.
- Lifecycle-managed activation and configurable reconnect/watchdog timing could
  improve managed bringup. Connection establishment still uses the SDK's bounded
  ten-second timeout; cancellation interrupts RPCs, not an SDK connection attempt.

References: [ROS QoS profiles](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html),
[REP 103](https://www.ros.org/reps/rep-0103.html),
[Blickfeld coordinate system](https://docs.blickfeld.com/qb2/Qb2/working_principles/coordinate_system.html).

## Observed sensor configuration after the firmware update

| Setting | Observed value |
| --- | --- |
| Named scan pattern | Dashboard |
| Horizontal × vertical FoV | 90° × 50° |
| Scanlines up / down | 100 / 34 |
| Frame mode | Up only |
| Target / actual frame rate | 4 / 4 Hz |
| Uniform scan / pulse mode | Enabled / C |
| Foveation | Disabled |
| Point-cloud filter | Firmware defaults (all returned filter scalars zero) |

With uniform scanning enabled, horizontal angle spacing is controlled by the
sensor's uniform pattern; the driver rejects direct angle-spacing updates in
that mode. This table records the baseline before tuning; afterward, the numeric
settings were restored and the active scan became custom/unnamed.
