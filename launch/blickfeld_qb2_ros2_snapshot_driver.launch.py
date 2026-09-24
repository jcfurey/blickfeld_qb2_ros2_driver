#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("blickfeld_qb2_ros2_driver"), "config", "snapshot.yaml"
        ]),
        description="YAML file containing the device lists and snapshot settings",
    )
    container = ComposableNodeContainer(
        name="blickfeld_qb2_component",
        namespace=LaunchConfiguration("namespace"),
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="blickfeld_qb2_ros2_driver",
                plugin="blickfeld::ros_interop::Qb2SnapshotDriver",
                name="blickfeld_qb2_snapshot_driver",
                namespace=LaunchConfiguration("namespace"),
                parameters=[LaunchConfiguration("params_file")],
                remappings=[("trigger_snapshot", "bf/trigger_snapshot")],
            ),
        ],
        output="screen",
    )
    return LaunchDescription([DeclareLaunchArgument("namespace", default_value=""), config, container])
