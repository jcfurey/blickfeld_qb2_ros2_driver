#include "utility/qb2_ros2_utils.h"

#include <grpc++/client_context.h>
#include <cstring>
#include <stdexcept>
#include <limits>

namespace blickfeld {
namespace ros_interop {

std::unique_ptr<sensor_msgs::msg::PointCloud2> convertToPointCloudMsg(const Qb2Frame& frame, const Qb2Info& qb2,
                                                                      std::optional<rclcpp::Time> timestamp) {
  auto point_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();

  /// if timestamp is passed use that otherwise get the time from frame
  if (timestamp) {
    point_cloud->header.stamp = *timestamp;
  } else {
    point_cloud->header.stamp = rclcpp::Time(frame.timestamp());
  }

  const auto number_of_points = frame.binary().length();
  const auto& binary = frame.binary();
  if (binary.cartesian().size() / (3 * sizeof(float)) < number_of_points ||
      (qb2.point_cloud_info.intensity && binary.photon_count().size() / sizeof(uint16_t) < number_of_points) ||
      (qb2.point_cloud_info.point_id && binary.direction_id().size() / sizeof(uint32_t) < number_of_points)) {
    throw std::invalid_argument("Qb2 frame binary fields are shorter than the declared point count");
  }

  point_cloud->fields.reserve(5);
  /// add fields to PointCloud2 msg
  addPointCloudField<float>(std::ref(*point_cloud), "x", point_cloud->point_step,
                            sensor_msgs::msg::PointField::FLOAT32);
  addPointCloudField<float>(std::ref(*point_cloud), "y", point_cloud->point_step,
                            sensor_msgs::msg::PointField::FLOAT32);
  addPointCloudField<float>(std::ref(*point_cloud), "z", point_cloud->point_step,
                            sensor_msgs::msg::PointField::FLOAT32);

  if (qb2.point_cloud_info.intensity == true) {
    /// HINT: To stick to Blickfeld PointCloud2 format the intensity is published as UINT32 and not as UINT16 that is
    /// published by Qb2 as photon count
    addPointCloudField<uint32_t>(std::ref(*point_cloud), "intensity", point_cloud->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
  }
  if (qb2.point_cloud_info.point_id == true) {
    addPointCloudField<uint32_t>(std::ref(*point_cloud), "point_id", point_cloud->point_step,
                                 sensor_msgs::msg::PointField::UINT32);
  }

  if (number_of_points > std::numeric_limits<uint32_t>::max() / point_cloud->point_step) {
    throw std::invalid_argument("Qb2 frame exceeds PointCloud2 row_step capacity");
  }
  /// reserve memory
  if (qb2.point_cloud_info.intensity || qb2.point_cloud_info.point_id)
    point_cloud->data.resize(number_of_points * point_cloud->point_step);

  /// set point cloud message data
  point_cloud->header.frame_id = qb2.point_cloud_info.frame_id;
  point_cloud->is_dense = false;
  point_cloud->height = 1;
  point_cloud->width = number_of_points;
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;

  // Qb2 binary fields are little endian. Copy XYZ and IDs byte-for-byte;
  // widening photon counts bytewise also works on big-endian hosts.
  point_cloud->is_bigendian = false;
  if (!qb2.point_cloud_info.intensity && !qb2.point_cloud_info.point_id) {
    point_cloud->data.assign(binary.cartesian().begin(),
                             binary.cartesian().begin() + point_cloud->row_step);
  } else {
    const size_t stride = point_cloud->point_step;
    const size_t id_offset = qb2.point_cloud_info.intensity ? 16 : 12;
    auto* destination = point_cloud->data.data();
    for (size_t i = 0; i < number_of_points; ++i, destination += stride) {
      std::memcpy(destination, binary.cartesian().data() + i * 12, 12);
      if (qb2.point_cloud_info.intensity) {
        std::memcpy(destination + 12, binary.photon_count().data() + i * 2, 2);
        // data.resize() initialized the high two bytes to zero.
      }
      if (qb2.point_cloud_info.point_id) {
        std::memcpy(destination + id_offset, binary.direction_id().data() + i * 4, 4);
      }
    }
  }
  return point_cloud;
}

std::string toString(CommunicationState state) {
  switch (state) {
    case CommunicationState::NOT_DEFINED:
      return "NOT_DEFINED";
      break;
    case CommunicationState::SUCCESS_READ:
      return "SUCCESS_READ";
      break;
    case CommunicationState::SUCCESS_OPEN_STREAM:
      return "SUCCESS_OPEN_STREAM";
      break;
    case CommunicationState::FAIL_READ:
      return "FAIL_READ";
      break;
    case CommunicationState::FAIL_NO_CONNECTION:
      return "FAIL_NO_CONNECTION";
      break;
    case CommunicationState::FAIL_EXCEPTION:
      return "FAIL_EXCEPTION";
      break;
    case CommunicationState::FAIL_AUTHENTICATION:
      return "FAIL_AUTHENTICATION";
      break;
    case CommunicationState::DISCONNECTED:
      return "DISCONNECTED";
      break;

    default:
      return "UNKNOWN";
      break;
  }
}

bool didAuthenticationFail(const grpc::Status& status) {
  return status.error_code() == grpc::StatusCode::UNAUTHENTICATED ||
         status.error_code() == grpc::StatusCode::PERMISSION_DENIED ||
         (std::string(status.error_message()).find("Authentication failed") != std::string::npos) ||
         (std::string(status.error_message()).find("Token renewal failed") != std::string::npos);
}

}  // namespace ros_interop
}  // namespace blickfeld
