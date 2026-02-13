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

#自作ライブラリ
from auto_robot_interfaces.action import OverSteps
from auto_robot_interfaces.msg import DynaFeedback, DynaTarget
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *

CAN_BUS = can.interface.Bus(bustype="socketcan",
                            channel="can0",
                            asynchronous=True,
                            bitrate=1000000)

# ----Config Params -----
TOF_THRESHOLD = 200  # (210 - 200 = 10) < 100[mm]


def calc_frame_height(dis_cm):
    return int((360 / (45 * math.pi) / 0.088) * (dis_cm))


def move_motor(pwm):
    send_packet_4byte(0x010, 3, -pwm, CAN_BUS)  # set_operating
    send_packet_4byte(0x011, 3, -pwm, CAN_BUS)  # set_operating
    send_packet_4byte(0x012, 3, pwm, CAN_BUS)  # set_operating
    send_packet_4byte(0x013, 3, pwm, CAN_BUS)  # set_operating


class OverStepsActionServer(Node):

    def __init__(self):
        super().__init__("over_steps_server")

        self._action_server = ActionServer(self,
                                           OverSteps,
                                           "over_steps",
                                           self.execute_callback,
                                           cancel_callback=self.cancel_callback)

        self.subscription_tof_forward = self.create_subscription(
            UInt16, "/tof_forward", self.subscribe_tof_forward, 10)
        self.subscription_tof_forward

        self.subscription_tof_backward = self.create_subscription(
            UInt16, "/tof_backward", self.subscribe_tof_backward, 10)
        self.subscription_tof_backward

        # publisherの設定
        # dynamixel_extended
        self.dyna_extpos_publisher = self.create_publisher(
            DynaTarget, "/dyna_target_extpos", 10)
        #twistの配信
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        #内部変数の定義
        self.tof_forward = 0
        self.tof_backward = 0

        send_packet_1byte(0x010, 0, 4, CAN_BUS)  # set_operating
        send_packet_1byte(0x011, 0, 4, CAN_BUS)  # set_operating
        send_packet_1byte(0x012, 0, 4, CAN_BUS)  # set_operating
        send_packet_1byte(0x013, 0, 4, CAN_BUS)  # set_operating

        #init_dynamixel
        self.publish_dyna_extpos(4, 600)
        self.publish_dyna_extpos(5, 600)

        #init_dc_motor
        move_motor(0)

    def publish_dyna_extpos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_extpos_publisher.publish(msg)

    def publish_twist(self, straight, w):
        twist = Twist()
        twist.linear.x = float(straight)
        twist.angular.z = float(w)
        self.twist_publisher.publish(twist)

    def subscribe_tof_forward(self, msg):
        self.tof_forward = msg.data

    def subscribe_tof_backward(self, msg):
        self.tof_backward = msg.data

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        self.get_logger().info("段差超えアクションを開始します。")

        feedback_msg = OverSteps.Feedback()
        result = OverSteps.Result()

        #フレーム上昇
        feedback_msg.inner_status = 0
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info("フレーム上昇")
        self.publish_dyna_extpos(4, calc_frame_height(230))
        self.publish_dyna_extpos(5, calc_frame_height(230))
        time.sleep(4.0)

        #直進
        feedback_msg.inner_status = 1
        goal_handle.publish_feedback(feedback_msg)
        self.publish_twist(0.1, 0)

        #段差検知待ち
        while self.tof_forward >= TOF_THRESHOLD:
            time.sleep(0.1)

        #停止して前方足上げ
        self.publish_twist(0, 0)
        move_motor(0)
        self.publish_dyna_extpos(4, calc_frame_height(-10))
        time.sleep(4.0)

        #再直進＆後方段差検知待ち
        self.publish_twist(0.1, 0)
        move_motor(400)
        while (self.tof_backward >= TOF_THRESHOLD):
            time.sleep(0.1)

        #停止して後方足上げ
        self.publish_twist(0, 0)
        move_motor(0)
        self.publish_dyna_extpos(5, calc_frame_height(-10))
        time.sleep(4.0)

        #最終直進
        self.publish_twist(0.1, 0)
        move_motor(400)
        time.sleep(2.0)

        #完了
        self.publish_twist(0, 0)
        move_motor(0)
        self.get_logger().info("段差超え完了")

        goal_handle.succeed()
        result.success = True
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info("キャンセルリクエスト送信")
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    node = OverStepsActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
