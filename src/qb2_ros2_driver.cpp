#include "qb2_ros2_driver.h"

#include <rclcpp/exceptions.hpp>
#include "utility/parameters.h"

#include <string>
#include <stdexcept>

namespace blickfeld {
namespace ros_interop {

Qb2Driver::Qb2Driver(rclcpp::NodeOptions options)
    : node_(std::make_shared<rclcpp::Node>("blickfeld_qb2_driver", options)),
      diagnostic_updater_(node_) {
  /// set parameters
  std::string fqdn = startupParameter<std::string>(*node_, "fqdn", "");
  std::string serial_number = startupParameter<std::string>(*node_, "serial_number", "");
  std::string application_key = startupParameter<std::string>(*node_, "application_key", "");

  application_key = applicationKey(application_key,
      startupParameter<std::string>(*node_, "application_key_file", ""));

  std::string system_unix_socket = startupParameter<std::string>(*node_, "system_unix_socket", "");

  std::string core_processing_unix_socket = startupParameter<std::string>(*node_, "core_processing_unix_socket", "");
  std::string frame_id = startupParameter<std::string>(*node_, "frame_id", "lidar");
  std::string point_cloud_topic = startupParameter<std::string>(*node_, "point_cloud_topic", "~/point_cloud_out");

  use_measurement_timestamp_ = startupParameter<bool>(*node_, "use_measurement_timestamp", use_measurement_timestamp_);
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "The 'use_measurement_timestamp' is set to: " << (use_measurement_timestamp_ ? "True" : "False"));

  bool intensity = startupParameter<bool>(*node_, "publish_intensity", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'publish_intensity' is set to: " << (intensity ? "True" : "False"));

  bool point_id = startupParameter<bool>(*node_, "publish_point_id", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'publish_point_id' is set to: " << (point_id ? "True" : "False"));

  /// figure out the host and method of connection
  if ((system_unix_socket.empty() || core_processing_unix_socket.empty()) && fqdn.empty()) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Neither unix socket nor fqdn were provided!");
    throw std::invalid_argument("Provide fqdn or both system_unix_socket and core_processing_unix_socket");
  }
  if (system_unix_socket.empty() != core_processing_unix_socket.empty()) {
    throw std::invalid_argument("Both Unix socket paths must be provided together");
  }
  if (!application_key.empty() && serial_number.empty()) {
    throw std::invalid_argument("serial_number is required when application_key is set");
  }

  Host host;
  /// logic to parse connection details, if both are provided, we'll prioritize unix socket
  if ((system_unix_socket.empty() && core_processing_unix_socket.empty()) == false) {
    host.system_socket = system_unix_socket;
    host.core_processing_socket = core_processing_unix_socket;
    host.connection_type = ConnectionType::UNIX_SOCKET;
  } else {
    host.fqdn = fqdn;
    host.connection_type = ConnectionType::FQDN;
  }
  host.serial_number = serial_number;
  host.application_key = application_key;

  const bool snapshot_mode = false;
  const float snapshot_frame_rate = 0.f;
  Qb2Info qb2_info{host, PointCloudInfo{frame_id, point_cloud_topic, intensity, point_id},
                   Snapshot{snapshot_mode, snapshot_frame_rate}};

  /// set the name of the driver as the hardwareID of the updater
  diagnostic_updater_.setHardwareID("Blickfeld Qb2 Driver");
  diagnostic_updater_.add(heartbeat_);
  stamp_status_ = std::make_shared<diagnostic_updater::TimeStampStatus>(diagnostic_updater::TimeStampStatusParam(),
                                                                        "Read frame time", node_->get_clock());
  diagnostic_updater_.add(*stamp_status_);

  auto point_cloud_reader = std::make_unique<qb2::PointCloudStreamer>(node_, qb2_info);
  qb2_ = std::make_unique<Qb2LidarRos>(node_, qb2_info, diagnostic_updater_, std::move(point_cloud_reader));

  /// launch thread for execution
  boost::asio::post(spin_thread_, [&]() { this->spinDriver(); });
}

Qb2Driver::~Qb2Driver() {
  is_running_ = false;
  qb2_->shutdownThreads();

  spin_thread_.stop();
  spin_thread_.join();
}

void Qb2Driver::spinDriver() {
  while (is_running_ && rclcpp::ok(node_->get_node_base_interface()->get_context())) {
    std::optional<Qb2Frame> frame = qb2_->readFrame();

    if (frame.has_value() == true && is_running_) {
      stamp_status_->tick(node_->now());
      qb2_->publishFrame(frame.value(),
                         use_measurement_timestamp_ ? rclcpp::Time(frame.value().timestamp()) : node_->now());
    }
  }
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Stopped grabbing frames from Blickfeld Qb2: " << qb2_->getQb2Info().hostName() << "!");
}

}  // namespace ros_interop
}  // namespace blickfeld

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blickfeld::ros_interop::Qb2Driver)
