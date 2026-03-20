"""
段上での制御フロー
"""

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
from auto_robot_interfaces_v2.action import MoveOnSteps
from dyna_interfaces.msg import DynaFeedback, DynaTarget
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *

# ----Config Params -----


def quaternion_to_yaw(x, y, w, z):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw


def limit_value(value, limit_value_abs):
    if (value > limit_value_abs):
        return limit_value_abs
    elif (value < -limit_value_abs):
        return -limit_value_abs

    return value


def calc_p(target, current, p_gain):
    p_value = (target - current) * p_gain
    p_value = limit_value(p_value, 0.5)

    return p_value


def is_in_threshold(target, current, threshold):
    e = target - current
    if (abs(e) < threshold):
        return True

    return False


class OnStepsController(Node):

    def __init__(self):
        super().__init__("on_steps_controller")
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            MoveOnSteps,
            "move_on_steps",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        # publisherの設定
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        #twistの配信
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        #odomの受取
        self.subscription = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.cb_group,
        )

        self.x = 0.00
        self.y = 0.00
        self.yaw = 0.00

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.w, q.z)

        self.x = pos.x
        self.y = pos.y
        self.yaw = math.degrees(yaw)

    def publish_twist(self, vx, vy, w):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(w)
        self.twist_publisher.publish(twist)

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = MoveOnSteps.Result()
        success = False

        if req.mode == 1:
            success = await self.go_right()

        elif req.mode == 2:
            success = await self.go_left()

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    async def go_right(self):

        target_x = 0.6
        target_y = 0.6

        self.get_logger().info("段上右動作を開始")
        #一定地点まで直進
        while not (is_in_threshold(target_x, self.x, 0.01)):
            p_value = calc_p(target_x, self.x, 0.5)
            self.publish_twist(p_value, 0, 0)
            time.sleep(0.1)

        #その場で右回り
        while not (is_in_threshold(-90.0, self.yaw, 1)):
            p_value = calc_p(-90.0, self.yaw, 0.05)
            self.publish_twist(0, 0, p_value)
            time.sleep(0.1)

        #一定地点まで直進
        while not (is_in_threshold(target_y, self.y, 0.01)):
            p_value = calc_p(target_y, self.y, 0.5)
            self.publish_twist(p_value, 0, 0)
            time.sleep(0.1)
        self.publish_twist(0, 0, 0)
        self.get_logger().info("動作終了、待機します")
        return True

    async def go_left(self):

        target_x = 0.6
        target_y = 0.6

        self.get_logger().info("段上左動作を開始")
        #一定地点まで直進
        while not (is_in_threshold(target_x, self.x, 0.01)):
            p_value = calc_p(target_x, self.x, 0.5)
            self.publish_twist(p_value, 0, 0)
            time.sleep(0.1)

        #その場で左回り
        while not (is_in_threshold(90.0, self.yaw, 1)):
            p_value = calc_p(90.0, self.yaw, 0.05)
            self.publish_twist(0, 0, p_value)
            time.sleep(0.1)

        #一定地点まで直進
        while not (is_in_threshold(target_y, self.y, 0.01)):
            p_value = calc_p(target_y, self.y, 0.5)
            self.publish_twist(p_value, 0, 0)
            time.sleep(0.1)

        self.publish_twist(0, 0, 0)
        self.get_logger().info("動作終了、待機します")

        return True


def main():
    rclpy.init()
    node = OnStepsController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
