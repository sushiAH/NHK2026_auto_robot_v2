import rclpy
import math
import numpy as np
import time

from std_msgs.msg import UInt8, UInt16, String
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import atexit

import asyncio
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

#自作ライブラリ
from auto_robot_interfaces_v2.action import DetectAruco
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from detect_aruco import *


# ----Config Params -----
class DetectArucoController(Node):

    def __init__(self):
        super().__init__("detect_aruco")

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            DetectAruco,
            "detect_aruco",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        self.marker_id = 1

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        self.get_logger().info("arucoマーカーを検出します")

        req = goal_handle.request
        res = DetectAruco.Result()
        success = False

        success = await detect_aruco(self.marker_id)

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res


def main():
    rclpy.init()
    node = DetectArucoController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
