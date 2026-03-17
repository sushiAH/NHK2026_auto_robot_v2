from setuptools import find_packages, setup
from setuptools import find_packages, setup
import os
from glob import glob

package_name = "auto_robot_v2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name,
                      "launch"), glob("launch/*_launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aratahorie",
    maintainer_email="aratahorie89@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest",],
    },
    entry_points={
        "console_scripts": [
            "subscribe_twist_node = auto_robot_v2.subscribe_twist_node:main",
            "follow_spline_node = auto_robot_v2.follow_spline_node:main",
            "joy2twist_node = auto_robot_v2.joy2twist_node:main",
            "publish_feedback_node = auto_robot_v2.publish_feedback_node:main",
            "control_over_steps_action_node = auto_robot_v2.control_over_steps_action_node:main",
            "control_box_arm_action_node = auto_robot_v2.control_box_arm_action_node:main",
            "control_spear_action_node = auto_robot_v2.control_spear_action_node:main",
            "correcting_pos_on_step_action_node = auto_robot_v2.correcting_pos_on_step_action_node:main",
            "switch_localization_action_node = auto_robot_v2.switch_localization_action_node:main",
            "move_on_steps_action_node = auto_robot_v2.move_on_steps_action_node:main",
            "detect_aruco_action_node = auto_robot_v2.detect_aruco_action_node:main",
            "is_vgoal_action_node = auto_robot_v2.is_vgoal_action_node:main",
            "robot_client_node = auto_robot_v2.robot_client_node:main",
        ],
    },
)
