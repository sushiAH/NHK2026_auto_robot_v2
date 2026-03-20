import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    package_dir = get_package_share_directory("auto_robot_v2")

    dyna_config_file_path = os.path.join(package_dir, "config",
                                         "dyna_params.yaml")

    # setting node
    oversteps_node = Node(
        package="auto_robot_v2",  # package_name
        executable="control_over_steps_action_node",  # node_name
        output="screen",
    )

    node2 = Node(
        package="auto_robot_v2",
        executable="publish_feedback_node",
    )

    node3 = Node(
        package="auto_robot_v2",
        executable="subscribe_twist_node",
    )

    dyna_node = Node(package="ah_ros2_dynamixel",
                     executable="dyna_handler_node_v2",
                     parameters=[dyna_config_file_path])

    node5 = Node(
        package="auto_robot_v2",
        executable="robot_client_node",
    )

    delayed_nodes = [oversteps_node]
    delayed_launch_node = TimerAction(period=3.0, actions=delayed_nodes)

    ld.add_action(dyna_node)
    ld.add_action(node2)
    ld.add_action(node3)
    ld.add_action(node5)
    ld.add_action(delayed_launch_node)

    return ld
