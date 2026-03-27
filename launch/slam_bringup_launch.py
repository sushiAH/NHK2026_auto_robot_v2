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
    #---- Params ----
    # SLAM用パラメータ
    slam_params = PathJoinSubstitution(
        [pkg_share, "config", "slam_params.yaml"])

    # ---- launchs  -----
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, '/launch/base_launch.py']))

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, '/launch/lidar_launch.py']))

    static_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_share, '/launch/static_tf_launch.py']))

    action_nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pkg_share, '/launch/action_nodes_launch.py']))

    # ----ノード定義----
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    slam_tool_box = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
        remappings=[("/scan", "/scan_filtered")],
    )

    ld.add_action(base_launch)
    ld.add_action(lidar_launch)
    ld.add_action(static_tf_launch)
    ld.add_action(action_nodes_launch)

    ld.add_action(joy_node)
    ld.add_action(slam_tool_box)

    return ld
