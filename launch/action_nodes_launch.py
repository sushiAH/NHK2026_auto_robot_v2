"""
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()
    pkg_share = FindPackageShare("auto_robot_v2")

    map_pure_pursuit_config_path = PathJoinSubstitution(
        [pkg_share, "config", "omni_pure_pursuit_map.yaml"])

    odom_pure_pursuit_config_path = PathJoinSubstitution(
        [pkg_share, "config", "omni_pure_pursuit_odom.yaml"])

    # ---- action node ----

    box_arm_node = Node(
        package="auto_robot_v2",
        executable="control_box_arm_action_node",
    )

    spear_node = Node(
        package="auto_robot_v2",
        executable="control_spear_action_node",
    )

    oversteps_node = Node(
        package="auto_robot_v2",
        executable="control_over_steps_action_node",
    )

    correcting_pos_node = Node(
        package="auto_robot_v2",
        executable="correcting_pos_on_step_action_node",
    )

    move_on_steps_node = Node(
        package="auto_robot_v2",
        executable="move_on_steps_action_node_v2",
    )

    detect_aruco_action_node = Node(
        package="auto_robot_v2",
        executable="detect_aruco_action_node",
    )

    is_vgoal_action_node = Node(
        package="auto_robot_v2",
        executable="is_vgoal_action_node",
    )

    switch_loc_node = Node(
        package="auto_robot_v2",
        executable="switch_localization_action_node",
    )

    pure_pursuit_map_node = Node(
        package="auto_robot_v2",
        executable="omni_pure_pursuit_action_node_v2",
        name="pure_pursuit_map",
        parameters=[map_pure_pursuit_config_path],
    )

    pure_pursuit_odom_node = Node(
        package="auto_robot_v2",
        executable="omni_pure_pursuit_action_node_v2",
        name="pure_pursuit_odom",
        parameters=[odom_pure_pursuit_config_path],
    )

    ld.add_action(oversteps_node)
    ld.add_action(box_arm_node)
    ld.add_action(spear_node)
    ld.add_action(correcting_pos_node)
    ld.add_action(move_on_steps_node)
    ld.add_action(detect_aruco_action_node)
    ld.add_action(is_vgoal_action_node)
    ld.add_action(switch_loc_node)

    ld.add_action(pure_pursuit_map_node)
    ld.add_action(pure_pursuit_odom_node)

    return ld
