#pragma once

#include "qb2_ros2_type.h"
#include <blickfeld_qb2_ros2_driver/srv/get_scan_pattern.hpp>
#include <blickfeld_qb2_ros2_driver/srv/set_scan_pattern.hpp>
#include <blickfeld_qb2_ros2_driver/srv/get_point_cloud_filter.hpp>
#include <blickfeld_qb2_ros2_driver/srv/set_point_cloud_filter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <mutex>

namespace blickfeld::ros_interop::qb2 {
namespace interfaces = blickfeld_qb2_ros2_driver;

// Independent channels and a mutually exclusive callback group serialize
// read/modify/write operations without blocking the cloud-reading thread.
class SensorConfiguration {
 public:
  SensorConfiguration(rclcpp::Node::SharedPtr node, const Qb2Info& info, const std::string& prefix);
  void getScanPattern(const interfaces::srv::GetScanPattern::Request& request,
                      interfaces::srv::GetScanPattern::Response& response);
  void setScanPattern(const interfaces::srv::SetScanPattern::Request& request,
                      interfaces::srv::SetScanPattern::Response& response);
  void getFilter(const interfaces::srv::GetPointCloudFilter::Request& request,
                 interfaces::srv::GetPointCloudFilter::Response& response);
  void setFilter(const interfaces::srv::SetPointCloudFilter::Request& request,
                 interfaces::srv::SetPointCloudFilter::Response& response);

 private:
  template <typename Response, typename Function>
  void handle(Response& response, Function function) {
    std::lock_guard<std::mutex> lock(mutex_);
    response = Response();
    deadline_ = std::chrono::system_clock::now() + std::chrono::seconds(5);
    try {
      function();
      response.success = true;
    } catch (const std::exception& error) {
      response.success = false;
      response.message = error.what();
    }
  }
  template <typename Function>
  void rpc(Function function) {
    grpc::ClientContext context;
    context.set_deadline(deadline_);
    const auto status = function(&context);
    if (!status.ok())
      throw std::runtime_error("Qb2 RPC " + std::to_string(status.error_code()) + ": " + status.error_message());
  }
  Qb2ChannelPtr channel(ConnectionTarget target);
  rclcpp::Node::SharedPtr node_;
  Qb2Info info_;
  std::mutex mutex_;
  std::chrono::system_clock::time_point deadline_;
  Qb2ChannelPtr system_channel_, core_channel_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::Service<interfaces::srv::GetScanPattern>::SharedPtr get_scan_;
  rclcpp::Service<interfaces::srv::SetScanPattern>::SharedPtr set_scan_;
  rclcpp::Service<interfaces::srv::GetPointCloudFilter>::SharedPtr get_filter_;
  rclcpp::Service<interfaces::srv::SetPointCloudFilter>::SharedPtr set_filter_;
};

}  // namespace blickfeld::ros_interop::qb2
