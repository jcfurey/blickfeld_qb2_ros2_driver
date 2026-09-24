#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    settings = [
        ("fqdn", "qb2", str, "Qb2 hostname or IP address"),
        ("serial_number", "", str, "Device serial number; required with an application key"),
        ("application_key", "", str, "Application key if device authentication is enabled"),
        ("frame_id", "lidar", str, "Point cloud TF frame"),
        ("point_cloud_topic", "/bf/points_raw", str, "Point cloud output topic"),
        ("use_measurement_timestamp", "false", bool, "Use device time; requires clock synchronization"),
        ("publish_intensity", "true", bool, "Include photon count as UINT32 intensity"),
        ("publish_point_id", "true", bool, "Include direction ID as UINT32 point_id"),
    ]
    arguments = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, _, description in settings
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type, _ in settings
    }
    container = ComposableNodeContainer(
        name="blickfeld_qb2_component",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="blickfeld_qb2_ros2_driver",
                plugin="blickfeld::ros_interop::Qb2Driver",
                name="blickfeld_qb2_driver",
                parameters=[parameters],
            ),
        ],
        output="screen",
    )
    return LaunchDescription(arguments + [container])
