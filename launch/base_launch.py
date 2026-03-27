"""
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

    # ---- Params ----
    ekf_config_file_path = os.path.join(package_dir, "config", "ekf.yaml")

    dyna_config_file_path = os.path.join(package_dir, "config",
                                         "dyna_params.yaml")

    # ノード定義
    sub_twist_node = Node(
        package="auto_robot_v2",
        executable="subscribe_twist_node",
    )

    pub_feedback_node = Node(
        package="auto_robot_v2",
        executable="publish_feedback_node",
    )

    joy2twist_node = Node(
        package="auto_robot_v2",
        executable="joy2twist_node",
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_config_file_path],
    )

    dyna_node = Node(
        package="ah_ros2_dynamixel",
        executable="dyna_handler_sync_node",
        parameters=[dyna_config_file_path],
    )

    ld.add_action(dyna_node)
    ld.add_action(sub_twist_node)
    ld.add_action(pub_feedback_node)
    ld.add_action(ekf_node)
    ld.add_action(joy2twist_node)

    return ld
