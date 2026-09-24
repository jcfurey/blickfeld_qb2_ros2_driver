cmake_minimum_required(VERSION 3.16)
project(blickfeld_qb2_ros2_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Options

# build against deployed qb2 protocols, defaults is ON
option(BUILD_DEPLOYED_PROTOCOL "Build ${CMAKE_PROJECT_NAME} deployed_protocol"
       ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find Boost dependencies
find_package(Boost 1.71 REQUIRED CONFIG)
find_package(Threads REQUIRED)

# find build tool dependencies
find_package(ament_cmake REQUIRED)

# find ROS2 dependencies
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ScanPattern.msg" "msg/PointCloudFilter.msg"
  "srv/GetScanPattern.srv" "srv/SetScanPattern.srv"
  "srv/GetPointCloudFilter.srv" "srv/SetPointCloudFilter.srv")
rosidl_get_typesupport_target(qb2_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_DEPLOYED_PROTOCOL)
  # fetch blickfeld API dependencies
  include(FetchContent)
  # Qb2 SDK v3.0.7. Pin the protocol so rebuilding cannot silently change the API.
  set(BLICKFELD_QB2_REVISION "181c2e06991155cec5b3a490a179ac6c9673eb9f"
      CACHE STRING "Blickfeld Qb2 SDK Git revision")
  FetchContent_Declare(blickfeld-qb2
    GIT_REPOSITORY https://github.com/Blickfeld/blickfeld-qb2.git
    GIT_TAG ${BLICKFELD_QB2_REVISION})
  FetchContent_MakeAvailable(blickfeld-qb2)
  set(blickfeld-protocol blickfeld-qb2)
else()
  # find internal dependencies needed for gRPC bindings
  find_package(blickfeld-base CONFIG REQUIRED fmt spdlog version gRPC)
  # needed for point cloud frame protobuf
  find_package(daedalus-core-processing REQUIRED)
  # needed for connecting to device
  find_package(daedalus-hardware REQUIRED)
  find_package(daedalus-system REQUIRED)
  set(blickfeld-protocol blickfeld-base daedalus-core_processing-protocol daedalus-hardware daedalus-system-protocol)
endif()

add_library(qb2_lidar_ros SHARED
  src/qb2_lidar_ros.cpp
  src/qb2/point_cloud_getter.cpp
  src/qb2/point_cloud_reader.cpp
  src/qb2/point_cloud_streamer.cpp
  src/qb2/sensor_configuration.cpp
  src/qb2/scan_pattern_watcher.cpp
  src/utility/qb2_connection_utils.cpp
  src/utility/qb2_ros2_utils.cpp
  )

target_include_directories(qb2_lidar_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(qb2_lidar_ros PUBLIC
  Boost::headers Threads::Threads rclcpp::rclcpp
  diagnostic_updater::diagnostic_updater
  diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
  rcl_interfaces::rcl_interfaces__rosidl_typesupport_cpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  ${blickfeld-protocol} ${qb2_typesupport})

add_library(qb2_ros2_driver SHARED src/qb2_ros2_driver.cpp)
target_link_libraries(qb2_ros2_driver PUBLIC qb2_lidar_ros rclcpp_components::component)
rclcpp_components_register_node(qb2_ros2_driver
  PLUGIN "blickfeld::ros_interop::Qb2Driver" EXECUTABLE qb2_driver)

add_library(qb2_ros2_snapshot_driver SHARED src/qb2_ros2_snapshot_driver.cpp)
target_link_libraries(qb2_ros2_snapshot_driver PUBLIC qb2_lidar_ros rclcpp_components::component
  std_srvs::std_srvs__rosidl_typesupport_cpp)
rclcpp_components_register_node(qb2_ros2_snapshot_driver
  PLUGIN "blickfeld::ros_interop::Qb2SnapshotDriver" EXECUTABLE qb2_snapshot_driver)

install(
  TARGETS qb2_lidar_ros qb2_ros2_driver qb2_ros2_snapshot_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install launch files
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# Install the profile client separately so it can be updated without replacing
# shared libraries used by a running component container.
install(PROGRAMS scripts/qb2_profile.py
  DESTINATION lib/${PROJECT_NAME} RENAME qb2_profile COMPONENT qb2_profile_tools)
install(FILES config/profiles.yaml
  DESTINATION share/${PROJECT_NAME}/config COMPONENT qb2_profile_tools)

# Testing
if(BUILD_TESTING)
  # Add gtests
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(tests)
endif()

ament_package()
