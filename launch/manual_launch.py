import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    package_dir = get_package_share_directory("auto_robot")

    ekf_config_file_path = os.path.join(
        get_package_share_directory("auto_robot"), "config", "ekf.yaml")

    # ノード定義
    odom_node = Node(
        package="auto_robot",  # package_name
        executable="publish_odom_node",  # node_name
        output="screen",
    )

    subtwist_node = Node(
        package="auto_robot",
        executable="subscribe_twist_node",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
    )

    joy2twist_node = Node(
        package="auto_robot",
        executable="joy2twist_node",
    )

    ekf_node = Node(package="robot_localization",
                    executable="ekf_node",
                    name="ekf_filter_node",
                    parameters=[ekf_config_file_path])

    pub_feed_node = Node(package="auto_robot",
                         executable="publish_feedback_node",
                         parameters=[{
                             "port_name": "/dev/ttyACM0",
                         }])

    dyna_node = Node(package="ah_ros2_dynamixel",
                     executable="dyna_handler_node",
                     parameters=[{
                         "port_name": "/dev/ttyUSB0",
                     }])

    ld.add_action(odom_node)
    ld.add_action(subtwist_node)
    ld.add_action(rviz_node)
    ld.add_action(joy2twist_node)
    ld.add_action(pub_feed_node)
    ld.add_action(dyna_node)

    return ld
