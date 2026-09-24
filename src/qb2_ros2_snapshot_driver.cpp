#include "qb2_ros2_snapshot_driver.h"

#include <google/protobuf/timestamp.pb.h>
#include <rclcpp/exceptions.hpp>
#include "utility/parameters.h"

#include <string>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace blickfeld {
namespace ros_interop {

Qb2SnapshotDriver::Qb2SnapshotDriver(rclcpp::NodeOptions options)
    : node_(std::make_shared<rclcpp::Node>("blickfeld_qb2_snapshot_driver", options)),
      diagnostic_updater_(node_) {
  /// setup parameters
  use_measurement_timestamp_ = startupParameter<bool>(*node_, "use_measurement_timestamp", use_measurement_timestamp_);
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "The 'use_measurement_timestamp' is set to: " << (use_measurement_timestamp_ ? "True" : "False"));

  bool intensity = startupParameter<bool>(*node_, "publish_intensity", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'publish_intensity' is set to: " << (intensity ? "True" : "False"));

  bool point_id = startupParameter<bool>(*node_, "publish_point_id", false);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'publish_point_id' is set to: " << (point_id ? "True" : "False"));

  const auto max_retries = startupParameter<int>(*node_, "max_retries", max_retries_);
  if (max_retries < 1 || max_retries > 255) {
    throw std::invalid_argument("max_retries must be between 1 and 255");
  }
  max_retries_ = static_cast<uint8_t>(max_retries);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'max_retries' is set to: " << max_retries);

  double snapshot_frame_rate = startupParameter<double>(*node_, "snapshot_frame_rate", 0.1);
  if (!std::isfinite(snapshot_frame_rate) || snapshot_frame_rate < min_allowed_snapshot_frame_rate_ ||
      snapshot_frame_rate > max_allowed_snapshot_frame_rate_) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "The 'snapshot_frame_rate' can only be in this range ["
                                                 << min_allowed_snapshot_frame_rate_ << ", "
                                                 << max_allowed_snapshot_frame_rate_ << "].");
    throw std::invalid_argument("snapshot_frame_rate must be between 0 and 0.1 Hz");
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'snapshot_frame_rate' is set to: " << snapshot_frame_rate);

  /// figure out the hosts and method of connection
  const std::vector<std::string> fqdns =
      startupParameter<std::vector<std::string>>(*node_, "fqdns", std::vector<std::string>());
  const std::vector<std::string> fqdn_serial_numbers =
      startupParameter<std::vector<std::string>>(*node_, "fqdn_serial_numbers", std::vector<std::string>());
  std::vector<std::string> fqdn_application_keys =
      startupParameter<std::vector<std::string>>(*node_, "fqdn_application_keys", std::vector<std::string>());
  const auto key_files = startupParameter<std::vector<std::string>>(
      *node_, "fqdn_application_key_files", {});
  if (!key_files.empty()) {
    if (key_files.size() != fqdns.size())
      throw std::invalid_argument("fqdn_application_key_files must match fqdns");
    if (fqdn_application_keys.empty()) fqdn_application_keys.resize(fqdns.size());
    if (fqdn_application_keys.size() != fqdns.size())
      throw std::invalid_argument("fqdn_application_keys must match fqdns");
    for (size_t i = 0; i < fqdns.size(); ++i)
      fqdn_application_keys[i] = applicationKey(fqdn_application_keys[i], key_files[i]);
  }
  const std::vector<std::string> fqdn_frame_ids =
      startupParameter<std::vector<std::string>>(*node_, "fqdn_frame_ids", std::vector<std::string>());
  const std::vector<std::string> fqdn_point_cloud_topics =
      startupParameter<std::vector<std::string>>(*node_, "fqdn_point_cloud_topics", std::vector<std::string>());

  const std::vector<std::string> system_unix_sockets =
      startupParameter<std::vector<std::string>>(*node_, "system_unix_sockets", std::vector<std::string>());
  const std::vector<std::string> core_processing_unix_sockets =
      startupParameter<std::vector<std::string>>(*node_, "core_processing_unix_sockets", std::vector<std::string>());
  const std::vector<std::string> unix_socket_frame_ids =
      startupParameter<std::vector<std::string>>(*node_, "unix_socket_frame_ids", std::vector<std::string>());
  const std::vector<std::string> unix_socket_point_cloud_topics =
      startupParameter<std::vector<std::string>>(*node_, "unix_socket_point_cloud_topics", std::vector<std::string>());

  if ((system_unix_sockets.empty() || core_processing_unix_sockets.empty()) && fqdns.empty()) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Neither unix socket nor fqdn were provided!");
    throw std::invalid_argument("Provide fqdns or both Unix socket lists");
  }

  const std::vector<size_t> fqdn_parameter_sizes{fqdns.size(), fqdn_serial_numbers.size(), fqdn_application_keys.size(),
                                                 fqdn_frame_ids.size(), fqdn_point_cloud_topics.size()};
  if (!std::all_of(fqdn_parameter_sizes.begin(), fqdn_parameter_sizes.end(),
                   [&](size_t size) { return size == fqdns.size(); })) {
    RCLCPP_FATAL_STREAM(node_->get_logger(), "Sizes of parameters don't match: fqdns: "
                                                 << fqdns.size() << ", fqdn_frame_ids: " << fqdn_frame_ids.size()
                                                 << ", fqdn_point_cloud_topics: " << fqdn_point_cloud_topics.size());
    throw std::invalid_argument("All five fqdn parameter lists must have the same length");
  }
  for (size_t i = 0; i < fqdns.size(); ++i) {
    if (!fqdn_application_keys[i].empty() && fqdn_serial_numbers[i].empty()) {
      throw std::invalid_argument("Each application key requires a corresponding serial number");
    }
  }

  const std::vector<size_t> socket_parameter_sizes{system_unix_sockets.size(), core_processing_unix_sockets.size(),
                                                   unix_socket_frame_ids.size(), unix_socket_point_cloud_topics.size()};
  if (!std::all_of(socket_parameter_sizes.begin(), socket_parameter_sizes.end(),
                   [&](size_t size) { return size == system_unix_sockets.size(); })) {
    RCLCPP_FATAL_STREAM(node_->get_logger(),
                        "Sizes of parameters don't match: system_unix_sockets: "
                            << system_unix_sockets.size()
                            << ", core_processing_unix_sockets: " << core_processing_unix_sockets.size()
                            << ", unix_socket_frame_ids: " << unix_socket_frame_ids.size()
                            << ", unix_socket_point_cloud_topics: " << unix_socket_point_cloud_topics.size());
    throw std::invalid_argument("All four Unix socket parameter lists must have the same length");
  }

  /// set the name of the driver as the hardwareID of the updater
  diagnostic_updater_.setHardwareID("Blickfeld Qb2 Snapshot Driver");
  diagnostic_updater_.add(heartbeat_);


  setupQb2Fqdns(fqdns, fqdn_serial_numbers, fqdn_application_keys, fqdn_frame_ids, fqdn_point_cloud_topics,
                snapshot_frame_rate, intensity, point_id);
  setupQb2UnixSockets(system_unix_sockets, core_processing_unix_sockets, unix_socket_frame_ids,
                      unix_socket_point_cloud_topics, snapshot_frame_rate, intensity, point_id);

  /// create a timer to trigger the snapshot with a fixed frame_rate only if the snapshot_frame_rate is not 0
  if (snapshot_frame_rate != 0) {
    std::chrono::duration<double> snapshot_duration(1. / snapshot_frame_rate);
    snapshot_timer_ =
        node_->create_wall_timer(snapshot_duration, [this](rclcpp::TimerBase&) { this->snapshotTriggerCallback(); });
  }

  /// setup a service to manually trigger the snapshot
  trigger_snapshot_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "trigger_snapshot", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        response->success = this->snapshotTriggerCallback();
        response->message = response->success ? "Snapshot queued" : "Snapshot busy or stopping";
      });

  // Manual mode (0 Hz) waits for the trigger service.
  if (snapshot_frame_rate > 0) {
    snapshotTriggerCallback();
  }
}

Qb2SnapshotDriver::~Qb2SnapshotDriver() {
  stopping_ = true;
  if (snapshot_timer_) snapshot_timer_->cancel();
  for (auto& qb2 : qb2s_) {
    qb2->shutdownThreads();
  }
  snapshot_thread_.stop();
  snapshot_thread_.join();
}

void Qb2SnapshotDriver::setupQb2Fqdns(const std::vector<std::string>& fqdns,
                                      const std::vector<std::string>& fqdn_serial_numbers,
                                      const std::vector<std::string>& fqdn_application_keys,
                                      const std::vector<std::string>& frame_ids,
                                      const std::vector<std::string>& point_cloud_topics, float snapshot_frame_rate,
                                      bool intensity, bool point_id) {
  for (size_t qb2_idx = 0; qb2_idx < fqdns.size(); ++qb2_idx) {
    Host host;
    host.connection_type = ConnectionType::FQDN;
    host.fqdn = fqdns[qb2_idx];
    host.serial_number = fqdn_serial_numbers[qb2_idx];
    host.application_key = fqdn_application_keys[qb2_idx];
    Qb2Info qb2_info{host, PointCloudInfo{frame_ids[qb2_idx], point_cloud_topics[qb2_idx], intensity, point_id},
                     Snapshot{snapshot_mode_, snapshot_frame_rate}};

    auto point_cloud_reader = std::make_unique<qb2::PointCloudGetter>(node_, qb2_info);
    qb2s_.emplace_back(
        std::make_unique<Qb2LidarRos>(node_, qb2_info, diagnostic_updater_, std::move(point_cloud_reader),
                                      "~/devices/device_" + std::to_string(qb2s_.size())));
  }
}

void Qb2SnapshotDriver::setupQb2UnixSockets(const std::vector<std::string>& system_unix_sockets,
                                            const std::vector<std::string>& core_processing_unix_sockets,
                                            const std::vector<std::string>& frame_ids,
                                            const std::vector<std::string>& point_cloud_topics,
                                            float snapshot_frame_rate, bool intensity, bool point_id) {
  for (size_t qb2_idx = 0; qb2_idx < system_unix_sockets.size(); ++qb2_idx) {
    Host host;
    host.connection_type = ConnectionType::UNIX_SOCKET;
    host.system_socket = system_unix_sockets[qb2_idx];
    host.core_processing_socket = core_processing_unix_sockets[qb2_idx];
    Qb2Info qb2_info{host, PointCloudInfo{frame_ids[qb2_idx], point_cloud_topics[qb2_idx], intensity, point_id},
                     Snapshot{snapshot_mode_, snapshot_frame_rate}};

    auto point_cloud_reader = std::make_unique<qb2::PointCloudGetter>(node_, qb2_info);
    qb2s_.emplace_back(
        std::make_unique<Qb2LidarRos>(node_, qb2_info, diagnostic_updater_, std::move(point_cloud_reader),
                                      "~/devices/device_" + std::to_string(qb2s_.size())));
  }
}

bool Qb2SnapshotDriver::snapshotTriggerCallback() {
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Try to get a snapshot if one is not running");
  if (stopping_) return false;
  if (!snapshot_is_running_.exchange(true)) {
    boost::asio::post(snapshot_thread_, [&]() { this->snapshot(); });
    return true;
  } else {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Failed to trigger a snapshot. The last snapshot is still running!");
    return false;
  }
}

void Qb2SnapshotDriver::snapshot() {
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Snapshot a frame from all Qb2s.");

  /// read all frames
  std::vector<Qb2Frame> frames;
  for (auto& qb2 : qb2s_) {
    std::optional<Qb2Frame> frame;
    uint8_t num_tries = 0;
    do {
      if (stopping_ || !rclcpp::ok(node_->get_node_base_interface()->get_context())) {
        snapshot_is_running_ = false;
        return;
      }
      frame = qb2->readFrame();
      num_tries++;
    } while (frame.has_value() == false && num_tries < max_retries_);

    if (frame.has_value() == false) {
      RCLCPP_WARN_STREAM(node_->get_logger(), "Failed reading the frame of Qb2: " << qb2->getQb2Info().hostName()
                                                                                  << ", Skipping the entire snapshot!");
      snapshot_is_running_ = false;
      return;
    }
    frames.emplace_back(std::move(frame.value()));
  }

  /// get one common timestamp to all frames
  rclcpp::Time snapshots_timestamp;
  if (use_measurement_timestamp_ == true) {
    const google::protobuf::uint64 oldest_timestamp =
        std::min_element(frames.begin(), frames.end(), [](const Qb2Frame& a, const Qb2Frame& b) {
          return a.timestamp() < b.timestamp();
        })->timestamp();
    snapshots_timestamp = rclcpp::Time(oldest_timestamp);
  } else {
    snapshots_timestamp = node_->now();
  }

  /// publish the frames on their respective topic
  int frame_index = 0;
  for (auto& qb2 : qb2s_) {
    if (stopping_) break;
    qb2->publishFrame(frames[frame_index], snapshots_timestamp);
    frame_index++;
  }
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Published " << qb2s_.size() << " point clouds.");
  snapshot_is_running_ = false;
}

}  // namespace ros_interop
}  // namespace blickfeld

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(blickfeld::ros_interop::Qb2SnapshotDriver)
