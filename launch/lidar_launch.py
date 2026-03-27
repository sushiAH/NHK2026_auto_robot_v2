"""
lidar launch
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

    lidar_launch_file_dir = os.path.join(
        get_package_share_directory("sllidar_ros2"), "launch")

    lidar_s2_launch_file_path = os.path.join(lidar_launch_file_dir,
                                             "sllidar_s2_launch.py")

    lidar_a3_launch_file_path = os.path.join(lidar_launch_file_dir,
                                             "sllidar_a3_launch.py")

    laser_filter_params = os.path.join(package_dir, "config",
                                       "laser_filter.yaml")

    # --- RPLIDAR S2 の設定 ---
    lidar_s2_setup_include = GroupAction(actions=[
        # 1. 名前空間を設定（これでトピックが /lidar_s2/scan になります）
        PushRosNamespace("lidar_s2"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_s2_launch_file_path),
            launch_arguments={
                "serial_port": "/dev/ttyUSB-S2",
                "serial_baudrate": "1000000",  # S2の標準
                "frame_id": "laser_s2",
                "scan_mode": "",  # 10Hz(同期用)
                "inverted": "true",  #逆さ向きならばtrue
            }.items(),
        ),
    ])

    # --- RPLIDAR A3 の設定 ---
    lidar_a3_setup_include = GroupAction(actions=[
        # 2. 名前空間を設定（これでトピックが /lidar_a3/scan になります）
        PushRosNamespace("lidar_a3"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lidar_a3_launch_file_path),
            launch_arguments={
                "serial_port": "/dev/ttyUSB-A3",
                "serial_baudrate": "256000",  # A3の標準
                "frame_id": "laser_a3",
                "scan_mode": "Sensitivity",  # 10Hz(同期用)
                "inverted": "true",
            }.items(),
        ),
    ])

    # lidar統合node
    scan_merger_node = Node(
        package="ros2_laser_scan_merger",
        executable="ros2_laser_scan_merger",
        name="laser_scan_merger",
        parameters=[{
            "pointCloudTopic": "merged_cloud",
            "pointCloutFrameId": "base_link",
            "scanTopic1": "/lidar_s2/scan",
            "scanTopic2": "/lidar_a3/scan",
            "show1": True,
            "show2": True,
            "laser1Alpha": -180.0,  #向き
            "laser1XOff": 0.225,  #x方向位置
            "laser1YOff": -0.250,  #y方向位置
            "laser1ZOff": 0.0,
            "laser2Alpha": 180.0,
            "laser2XOff": 0.225,
            "laser2YOff": 0.250,
            "laser2ZOff": 0.0,
            "laser1AngleMax": 180.0,
            "laser1AngleMin": -180.0,
            "laser2AngleMax": 180.0,
            "laser2AngleMin": -180.0,
        }],
    )

    # pointCloud -> scan
    pc_to_scan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[{
            "target_frame": "base_link",
            "transform_tolerance": 0.01,
            "min_height": -1.0,
            "max_height": 1.0,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0043,  # 約0.25度（LiDARの性能に合わせる）
            "scan_time": 0.1,
            "range_min": 0.2,
            "range_max": 30.0,
            "use_inf": True,
        }],
        remappings=[
            ("cloud_in", "/merged_cloud"),  # 入力
            ("scan", "/scan_merged_raw"),  # 出力 
        ],
    )

    # laser filter node
    # #Lidar(/scan) -> Filter ->/scan_filtered
    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            laser_filter_params, {
                "qos_overrides./scan.reliability": "best_effort"
            }
        ],
        output="screen",
        remappings=[
            ("scan", "/scan_merged_raw"),  # 入力　
            ("scan_filtered", "/scan_filtered")  # 出力
        ],
    )

    ld.add_action(lidar_s2_setup_include)
    ld.add_action(lidar_a3_setup_include)
    ld.add_action(scan_merger_node)
    ld.add_action(pc_to_scan_node)
    ld.add_action(laser_filter_node)

    return ld
