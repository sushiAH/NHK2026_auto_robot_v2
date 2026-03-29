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
    map_file_path = PathJoinSubstitution([pkg_share, "map", "c419_map.yaml"])

    nav2_params_path = PathJoinSubstitution(
        [pkg_share, "config", "nav2_params.yaml"])

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

    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    record_path_node = Node(
        package="auto_robot_v2",
        executable="record_path_node",
        parameters=[{
            "global_frame": "map",
        }],
    )

    # map server node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[nav2_params_path, {
            "yaml_filename": map_file_path
        }],
    )

    # amcl
    amcl_node = Node(package="nav2_amcl",
                     executable="amcl",
                     parameters=[nav2_params_path])

    # life cycle node
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_control",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server", "amcl"],
            "bond_timeout": 0.0,
        }],
    )

    ld.add_action(base_launch)
    ld.add_action(lidar_launch)
    ld.add_action(static_tf_launch)
    ld.add_action(action_nodes_launch)

    ld.add_action(joy_node)
    ld.add_action(record_path_node)
    ld.add_action(amcl_node)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)

    return ld
