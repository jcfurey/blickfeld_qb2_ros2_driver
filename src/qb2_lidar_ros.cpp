#include "qb2_lidar_ros.h"

#include "utility/qb2_ros2_utils.h"

#include <blickfeld/base/grpc_defs.h>

#include <grpc++/create_channel.h>
#include <diagnostic_updater/publisher.hpp>

#include <optional>
#include <utility>

namespace blickfeld {
namespace ros_interop {

Qb2LidarRos::Qb2LidarRos(rclcpp::Node::SharedPtr node, const Qb2Info& qb2,
                         diagnostic_updater::Updater& diagnostic_updater,
                         std::unique_ptr<qb2::PointCloudReader> point_cloud_reader,
                         const std::string& service_prefix)
    : node_(node), qb2_(qb2), driver_status_(qb2), point_cloud_reader_(std::move(point_cloud_reader)) {
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'host' is set to: " << qb2_.hostName());
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'frame_id' is set to: " << qb2_.point_cloud_info.frame_id);
  RCLCPP_INFO_STREAM(node_->get_logger(), "The 'point_cloud_topic' is set to: " << qb2_.point_cloud_info.topic);

  /// diagnostic updater
  diagnostic_updater.add("Diagnostic message - " + qb2_.hostName(), &driver_status_, &DriverStatus::updateDiagnostic);

  scan_pattern_watcher_ = std::make_unique<qb2::ScanPatternWatcher>(node, qb2);

  // Bounds are refreshed only in the diagnostic callback, avoiding a race
  // between the scan-pattern worker and FrequencyStatus's double pointers.
  const int window_size = qb2_.snapshot.mode && qb2_.snapshot.frame_rate > 0
                              ? static_cast<int>(std::ceil((1 / qb2_.snapshot.frame_rate + 10) /
                                                         diagnostic_updater.getPeriod().seconds())) : 5;
  frequency_status_ = std::make_unique<diagnostic_updater::FrequencyStatus>(
      diagnostic_updater::FrequencyStatusParam(&frequency_, &frequency_, frequency_tolerance_,
                                               std::max(1, window_size)),
      std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME));
  timestamp_status_ = std::make_unique<diagnostic_updater::TimeStampStatus>(
      diagnostic_updater::TimeStampStatusParam(), node_->get_clock());
  rclcpp::PublisherOptions publisher_options;
  publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
  // Preserve reliable/depth-1 defaults; sensor-data QoS can be selected through
  // ROS's standard qos_overrides parameters without recompilation.
  point_cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      qb2_.point_cloud_info.topic, rclcpp::QoS(1), publisher_options);
  diagnostic_updater.add(std::string(point_cloud_publisher_->get_topic_name()) + " topic status",
                         this, &Qb2LidarRos::updateTopicDiagnostic);

  sensor_configuration_ = std::make_unique<qb2::SensorConfiguration>(node_, qb2_, service_prefix);

  is_running_ = true;
  /// thread to make sure connection is correctly established
  boost::asio::post(worker_thread_pool_, [&]() { this->checkForHangingStreams(); });
  /// thread for running the scan pattern watch
  boost::asio::post(worker_thread_pool_, [&]() { this->watchScanPattern(); });
}

Qb2LidarRos::~Qb2LidarRos() {
  shutdownThreads();

  worker_thread_pool_.stop();
  worker_thread_pool_.join();
}

void Qb2LidarRos::shutdownThreads() {
  {
    std::lock_guard<std::mutex> guard(is_running_mutex_);
    is_running_ = false;
  }
  point_cloud_reader_->cancel(true);
  scan_pattern_watcher_->cancel(true);
  is_running_condition_variable_.notify_all();
}

const Qb2Info& Qb2LidarRos::getQb2Info() const { return qb2_; }

std::optional<Qb2Frame> Qb2LidarRos::readFrame() {
  std::optional<Qb2Frame> frame;
  auto request_api_timestamp = std::chrono::steady_clock::now();
  CommunicationState communication_state = CommunicationState::NOT_DEFINED;
  std::tie(communication_state, frame) = point_cloud_reader_->readFrame();
  auto received_api_timestamp = std::chrono::steady_clock::now();

  uint64_t frame_id = frame.has_value() ? frame.value().id() : 0;
  driver_status_.updateRuntimeStatus(frame_id, communication_state, request_api_timestamp, received_api_timestamp);

  if (qb2::hasQb2CommunicationFailed(communication_state) == true) {
    interruptableSleep();
  }
  return frame;
}

void Qb2LidarRos::publishFrame(const Qb2Frame& frame, rclcpp::Time timestamp) {
  std::unique_ptr<sensor_msgs::msg::PointCloud2> point_cloud;
  try {
    point_cloud = convertToPointCloudMsg(frame, qb2_, timestamp);
  } catch (const std::invalid_argument& error) {
    RCLCPP_WARN(node_->get_logger(), "Dropping invalid Qb2 frame: %s", error.what());
    driver_status_.onInvalidFrame();
    return;
  }
  timestamp_status_->tick(point_cloud->header.stamp);
  frequency_status_->tick();
  point_cloud_publisher_->publish(std::move(point_cloud));

  driver_status_.onPublishingFrame();
  ++published_frames_;
}

void Qb2LidarRos::disconnect() {
  /// HINT: these cancel calls are needed to interrupt the streams and tell the server that we are closing the
  /// connection
  point_cloud_reader_->cancel();
  scan_pattern_watcher_->cancel();

  driver_status_.disconnect();
}

void Qb2LidarRos::checkForHangingStreams() {
  while (is_running_ == true) {
    if (driver_status_.isFrameTooOld() == true) {
      RCLCPP_WARN_STREAM(node_->get_logger(),
                         "Last frame from Qb2: " << qb2_.hostName() << " is too old, disconnecting!");
      disconnect();
    }
    interruptableSleep();
  }
}

void Qb2LidarRos::watchScanPattern() {
  while (is_running_ == true) {
    std::optional<Qb2ScanPattern> scan_pattern;
    CommunicationState communication_state = CommunicationState::NOT_DEFINED;
    std::tie(communication_state, scan_pattern) = scan_pattern_watcher_->watchScanPattern();

    driver_status_.updateScanPattern(scan_pattern, communication_state);

    if (qb2::hasQb2CommunicationFailed(communication_state) == false) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Scan Pattern changed!");
      RCLCPP_INFO_STREAM(node_->get_logger(), "Target frame rate: " << driver_status_.getTargetFrameRate());
    } else {
      interruptableSleep();
    }
  }
}

void Qb2LidarRos::interruptableSleep() {
  std::unique_lock<std::mutex> lock(is_running_mutex_);
  if (is_running_ == true) {
    is_running_condition_variable_.wait_for(lock, std::chrono::seconds(base::GRPC_DEFAULT_CONNECTION_TIMEOUT));
  }
}

void Qb2LidarRos::updateTopicDiagnostic(diagnostic_updater::DiagnosticStatusWrapper& status) {
  std::lock_guard<std::mutex> lock(diagnostic_mutex_);
  const double rate = driver_status_.getTargetFrameRate();
  if (qb2_.snapshot.mode && rate == 0) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Manual snapshots; no periodic rate expected");
  } else if (!std::isfinite(rate) || rate <= 0) {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for scan pattern");
    return;
  } else {
    if (frequency_ != rate) frequency_status_->clear();
    frequency_ = rate;
    frequency_status_->run(status);
  }
  const auto frames = published_frames_.load();
  if (frames == last_diagnosed_frames_) return;
  last_diagnosed_frames_ = frames;
  diagnostic_updater::DiagnosticStatusWrapper timestamp_status;
  timestamp_status_->run(timestamp_status);
  status.mergeSummary(timestamp_status);
  status.values.insert(status.values.end(), timestamp_status.values.begin(), timestamp_status.values.end());
}

}  // namespace ros_interop
}  // namespace blickfeld
