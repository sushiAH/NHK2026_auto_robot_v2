import os
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, SetEnvironmentVariable


def generate_launch_description():
    ld = LaunchDescription()
    package_dir = get_package_share_directory("auto_robot_v2")

    # ----Params----
    slam_params = os.path.join(package_dir, "config", "slam_params.yaml")

    ekf_config_file_path = os.path.join(package_dir, "config", "ekf.yaml")

    # laser_filter用パラメータ
    laser_filter_params = os.path.join(package_dir, "config",
                                       "laser_filter.yaml")

    dyna_config_file_path = os.path.join(package_dir, "config",
                                         "dyna_params.yaml")

    zenoh_bridge = ExecuteProcess(
        cmd=[
            "zenoh-bridge-ros2dds",
            "-d",
            "1",
            "--rest-http-port",
            "8000",
        ],
        name="zenoh_bridge",
    )

    # ----ノード定義----
    move_on_steps_node = Node(
        package="auto_robot_v2",
        executable="move_on_steps_action_node",
    )

    box_arm_node = Node(
        package="auto_robot_v2",
        executable="control_box_arm_action_node",
    )

    correct_pos_on_steps_node = Node(
        package="auto_robot_v2",
        executable="correcting_pos_on_step_action_node",
    )

    oversteps_node = Node(
        package="auto_robot_v2",
        executable="control_over_steps_action_node",
    )

    robot_client_node = Node(
        package="auto_robot_v2",
        executable="robot_client_node",
    )

    subtwist_node = Node(
        package="auto_robot_v2",
        executable="subscribe_twist_node",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
    )

    joy2twist_node = Node(
        package="auto_robot_v2",
        executable="joy2twist_node",
    )

    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
    )

    ekf_node = Node(package="robot_localization",
                    executable="ekf_node",
                    name="ekf_filter_node",
                    parameters=[ekf_config_file_path])

    pub_feed_node = Node(package="auto_robot_v2",
                         executable="publish_feedback_node",
                         parameters=[{
                             "port_name": "/dev/ttyACM0",
                         }],
                         output="screen")

    dyna_node = Node(package="ah_ros2_dynamixel",
                     executable="dyna_handler_node_v2",
                     parameters=[dyna_config_file_path])

    static_transform_publisher_footprint_node = Node(
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

    delayed_nodes = [oversteps_node]
    delayed_launch_node = TimerAction(period=3.0, actions=delayed_nodes)

    ld.add_action(dyna_node)
    #ld.add_action(correct_pos_on_steps_node)
    #ld.add_action(move_on_steps_node)
    #ld.add_action(box_arm_node)

    #ld.add_action(zenoh_bridge)
    ld.add_action(subtwist_node)
    ld.add_action(rviz_node)
    ld.add_action(joy2twist_node)
    ld.add_action(pub_feed_node)
    ld.add_action(ekf_node)
    ld.add_action(joy_node)

    ld.add_action(static_transform_publisher_imu_node)
    ld.add_action(static_transform_publisher_footprint_node)
    ld.add_action(delayed_launch_node)

    #ld.add_action(robot_client_node)

    return ld
