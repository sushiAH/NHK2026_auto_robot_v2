import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    package_dir = get_package_share_directory("auto_robot")

    # setting node
    node1 = Node(
        package="auto_robot",  # package_name
        executable="control_over_steps_node",  # node_name
        output="screen",
    )

    node2 = Node(
        package="auto_robot",
        executable="publish_feedback_node",
    )

    node3 = Node(
        package="auto_robot",
        executable="twist_subscriber_auto_node",
    )

    node4 = Node(
        package="ah_ros2_dynamixel",
        executable="dyna_handler",
    )

    ld.add_action(node4)
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node1)

    return ld
