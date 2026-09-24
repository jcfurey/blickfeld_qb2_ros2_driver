#pragma once
#include "qb2_ros2_type.h"

namespace blickfeld::ros_interop {
// Only pre-actual firmware needs these deprecated protobuf accessors.
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
inline double actualFrameRate(const system::config::ScanPattern& scan) {
  const auto& rate = scan.frame_rate();
  return rate.has_actual() ? rate.actual() : rate.maximum();
}
inline void clearReadOnlyScanFields(system::config::ScanPattern& scan) {
  scan.clear_read_only();
  scan.mutable_frame_rate()->clear_actual();
  scan.mutable_frame_rate()->clear_maximum();
  if (scan.vertical().has_foveation()) scan.mutable_vertical()->mutable_foveation()->clear_scanlines();
}
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif
}  // namespace blickfeld::ros_interop
