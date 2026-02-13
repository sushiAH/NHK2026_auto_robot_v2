from setuptools import find_packages, setup
from setuptools import find_packages, setup
import os
from glob import glob

package_name = "auto_robot"

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
            "subscribe_twist_node = auto_robot.subscribe_twist_node:main",
            "publish_odom_node = auto_robot.publish_odom_node:main",
            "follow_spline_node = auto_robot.follow_spline_node:main",
            "joy2twist_node = auto_robot.joy2twist_node:main",
            "control_box_arm_node = auto_robot.control_box_arm_node:main",
            "control_over_steps_node = auto_robot.control_over_steps_node:main",
            "publish_feedback_node = auto_robot.publish_feedback_node:main",
            "control_over_steps_action_node = auto_robot.control_over_steps_action_node:main",
        ],
    },
)
