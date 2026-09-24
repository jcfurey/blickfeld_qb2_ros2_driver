#pragma once

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace blickfeld::ros_interop {

template <typename T>
T startupParameter(rclcpp::Node& node, const std::string& name, const T& default_value) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.description = "Startup setting; restart the component to change it.";
  return node.declare_parameter<T>(name, default_value, descriptor);
}

inline std::string applicationKey(const std::string& value, const std::string& path) {
  if (path.empty()) return value;
  if (!value.empty()) throw std::invalid_argument("Use application_key or application_key_file, not both");
  std::ifstream stream(path);
  if (!stream) throw std::invalid_argument("Cannot read application key file");
  std::string key{std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
  if (stream.bad()) throw std::invalid_argument("Failed reading application key file");
  const auto begin = key.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) throw std::invalid_argument("Application key file is empty");
  key = key.substr(begin, key.find_last_not_of(" \t\r\n") - begin + 1);
  if (key.find_first_of(" \t\r\n") != std::string::npos)
    throw std::invalid_argument("Application key file must contain one token");
  return key;
}

}  // namespace blickfeld::ros_interop
