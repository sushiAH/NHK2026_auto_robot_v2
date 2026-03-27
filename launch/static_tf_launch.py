"""
static tf launch
base_link->s2
base_link->a3
base_link->footprint
base_link->imu_link
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory("auto_robot_v2")
    # ---- 静的tfの配信----
    static_tf_s2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.225",  #x
            "-0.250",  #y
            "0.0",  #z
            "-3.1416",  #yaw
            "0.0",  #pitch
            "0.0",  #roll
            "base_link",  #parent frame id
            "laser_s2",  #child frame id
        ],
    )

    static_tf_a3 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.225",
            "0.250",
            "0.0",
            "3.1416",
            "0.0",
            "0.0",
            "base_link",
            "laser_a3",
        ],
    )

    # base_link -> base_footprint
    static_transform_publisher_footprint_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            "base_footprint",
        ],
    )

    static_transform_publisher_imu_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            "imu_link",
        ])

    ld.add_action(static_tf_a3)
    ld.add_action(static_tf_s2)
    ld.add_action(static_transform_publisher_imu_node)
    ld.add_action(static_transform_publisher_footprint_node)

    return ld
