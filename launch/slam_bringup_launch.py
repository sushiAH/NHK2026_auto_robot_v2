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

    dyna_config_file_path = PathJoinSubstitution(
        [pkg_share, "config", "dyna_params.yaml"])

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
    dyna_node = Node(
        package="ah_ros2_dynamixel",
        executable="dyna_handler_sync_node",
        parameters=[dyna_config_file_path],
    )

    delayed_nodes = [base_launch, lidar_launch, static_tf_launch]
    delayed_launch_node = TimerAction(period=3.0, actions=delayed_nodes)

    action_nodes = [action_nodes_launch]
    action_nodes = TimerAction(period=2.0, actions=action_nodes)

    ld.add_action(dyna_node)
    ld.add_action(delayed_launch_node)
    ld.add_action(action_nodes)
    ld.add_action(joy_node)
    ld.add_action(slam_tool_box)

    return ld
