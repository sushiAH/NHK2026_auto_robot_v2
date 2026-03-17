"""tofセンサーで機体下から地面の距離を計測。
Vgoal達成可能高さまで待機する。"""

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
from auto_robot_interfaces_v2.action import IsVgoal
import os
import sys


# ----Config Params -----
class IsVgoalChecker(Node):

    def __init__(self):
        super().__init__("is_vgoal")

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            IsVgoal,
            "is_vgoal",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        #Vgoal用tofセンサー
        self.subscription_tof_forward = self.create_subscription(
            UInt16,
            "/tof_forward",
            self.subscribe_tof_forward,
            10,
            callback_group=self.cb_group,
        )
        self.subscription_tof_forward

        self.tof_forward = 0

        #Vgoalしきい値センサー
        self.vgoal_height_threshold = 0

    def subscribe_tof_forward(self, msg):
        self.tof_forward = msg.data

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        self.get_logger().info("Vゴールを待機します")

        req = goal_handle.request

        res = IsVgoal.Result()
        success = False

        success = await self.is_vgoal()

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    async def is_vgoal(self):
        while (self.tof_forward < self.vgoal_height_threshold):
            time.sleep(0.1)
        return True


def main():
    rclpy.init()
    node = IsVgoalChecker()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
