"""段差上り、降り制御フロー
段差上り
    初期化時、昇降を上げる(10,-10) [mm]
    昇降下げる(-210,210) [mm]
    前方に進める
    tof1の値が小さくなったら(段差が近づいたら)、車体を止めて、前方を上げる(10,210) [mm]
    決まった秒数が立ったら、dcモーターと、dynamixelを前方に動かす
    tof_の値が小さくなったら(段差が近づいたら)、車体を止めて、後方を上げる(10,-10) [mm]
    dcモーターを前方に動かす
    決まった秒数が立ったら終了

段差降り(後ろ向き)
    最初は(10,-10) [mm]の状態
    前方に直進する
    tof2がしきい値を超えたら、車体を止めて、前方を下げる(-210[mm],-10)
    決まった秒数が立ったら、dcモーターを前方に動かす
    tof4がしきい値を超えたら、車体を止めて、後方を下げる(-210[mm],210[mm])
    dynamixelとdcモーターを前方に進める
    決まった秒数が立ったら、車体を停止して、フレームを(10,-10) [mm]とする
    決まった秒数が立ったら終了
　　
"""

import rclpy
import math
import numpy as np
import time

from std_msgs.msg import UInt8, UInt16, String
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import atexit

import asyncio
from rclpy.action import ActionServer, CancelResponse

#自作ライブラリ
from auto_robot_interfaces_v2.action import OverSteps
from dyna_interfaces.msg import DynaFeedback, DynaTarget
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
TOF_THRESHOLD_CLIMB = 200  # (210 - 200 = 10) < 100[mm]
TOF_THRESHOLD_DESCEND = 150


def calc_frame_height(dis_cm):
    return int((360 / (45 * math.pi) / 0.088) * (dis_cm))


class OverStepsActionServer(Node):

    def __init__(self):
        super().__init__("over_steps_server")

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            OverSteps,
            "over_steps",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        self.subscription_tof1 = self.create_subscription(
            UInt16,
            "/tof_1",
            self.subscribe_tof1,
            10,
            callback_group=self.cb_group,
        )
        self.subscription_tof1

        self.subscription_tof2 = self.create_subscription(
            UInt16,
            "/tof_2",
            self.subscribe_tof2,
            10,
            callback_group=self.cb_group,
        )
        self.subscription_tof2

        self.subscription_tof3 = self.create_subscription(
            UInt16,
            "/tof_3",
            self.subscribe_tof3,
            10,
            callback_group=self.cb_group,
        )
        self.subscription_tof3

        self.subscription_tof4 = self.create_subscription(
            UInt16,
            "/tof_4",
            self.subscribe_tof4,
            10,
            callback_group=self.cb_group,
        )
        self.subscription_tof4

        # publisherの設定
        # dynamixel_extended
        self.dyna_extpos_publisher = self.create_publisher(
            DynaTarget, "/dyna_target_extpos", 10)

        self.dyna_vel_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_vel", 10)

        #dc_motor_twistの配信
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        #内部変数の定義
        self.tof1 = 0
        self.tof2 = 0
        self.tof3 = 0
        self.tof4 = 0

        #init_dynamixel
        self.publish_dyna_extpos(2, calc_frame_height(10))  # 前方上げ
        self.publish_dyna_extpos(3, calc_frame_height(-10))  # 後方上げ

    def publish_dyna_extpos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_extpos_publisher.publish(msg)

    def publish_dyna_vel(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_vel_publisher.publish(msg)

    def publish_dyna_twist(self, vx):
        self.publish_dyna_vel(0, -vx)
        self.publish_dyna_vel(1, vx)

    def publish_twist(self, vx, vy, w):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(w)
        self.twist_publisher.publish(twist)

    def subscribe_tof1(self, msg):
        self.tof1 = msg.data

    def subscribe_tof2(self, msg):
        self.tof2 = msg.data

    def subscribe_tof3(self, msg):
        self.tof3 = msg.data

    def subscribe_tof4(self, msg):
        self.tof4 = msg.data

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = OverSteps.Result()
        success = False

        if req.mode == 1:
            success = await self.climb_steps()
        elif req.mode == 2:
            success = await self.descend_steps()

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    async def climb_steps(self):
        self.get_logger().info("段差超えアクションを開始します。")

        #フレーム上昇
        self.get_logger().info("フレーム上昇")
        self.publish_dyna_extpos(2, calc_frame_height(-300))
        self.publish_dyna_extpos(3, calc_frame_height(300))
        time.sleep(4.0)

        #dynamixel前進
        self.publish_dyna_twist(100)

        #段差検知待ち
        while self.tof1 >= TOF_THRESHOLD_CLIMB:
            time.sleep(0.1)

        #停止して前方足上げ
        self.publish_dyna_twist(0)
        self.publish_dyna_extpos(2, calc_frame_height(10))
        time.sleep(4.0)

        #再直進＆後方段差検知待ち
        self.publish_dyna_twist(100)
        self.publish_twist(0.5, 0, 0)
        while (self.tof3 >= TOF_THRESHOLD_CLIMB):
            time.sleep(0.1)

        #停止して後方足上げ
        self.publish_dyna_twist(0)
        self.publish_twist(0, 0, 0)
        self.publish_dyna_extpos(3, calc_frame_height(-10))
        time.sleep(4.0)

        #最終直進
        self.publish_dyna_twist(100)
        self.publish_twist(0.5, 0, 0)
        time.sleep(2.0)

        #停止
        self.publish_dyna_twist(0)
        self.publish_twist(0, 0, 0)

        return True

    async def descend_steps(self):

        self.get_logger().info("段差降りアクションを開始します。")

        self.publish_dyna_extpos(2, calc_frame_height(-10))
        self.publish_dyna_extpos(3, calc_frame_height(10))
        time.sleep(1.0)

        #前進
        self.publish_twist(0.5, 0, 0)
        self.publish_dyna_twist(100)

        #tof2検知待ち
        while (self.tof2 <= TOF_THRESHOLD_DESCEND):
            time.sleep(0.1)

        #車体を止めて、前方の足を下げる
        self.publish_dyna_twist(0)
        self.publish_twist(0, 0, 0)
        self.publish_dyna_extpos(2, calc_frame_height(-300))
        time.sleep(4.0)

        #dcモーターを前方に動かす
        self.publish_twist(0.5, 0, 0)
        self.publish_dyna_twist(100)

        #tof4検知待ち
        while (self.tof4 <= TOF_THRESHOLD_DESCEND):
            time.sleep(0.1)

        #車体を止めて、後方の足を下げる
        self.publish_twist(0, 0, 0)
        self.publish_dyna_twist(0)
        self.publish_dyna_extpos(3, calc_frame_height(300))
        time.sleep(4.0)

        #前向きに直進する
        self.publish_twist(0.5, 0, 0)
        self.publish_dyna_twist(100)
        time.sleep(2.0)

        #車体を停止し、フレーム高さをもとに戻す
        self.publish_twist(0, 0, 0)
        self.publish_dyna_twist(0)
        self.publish_dyna_extpos(2, calc_frame_height(10))
        self.publish_dyna_extpos(3, calc_frame_height(-10))
        time.sleep(2.0)

        return True


def main():
    rclpy.init()
    node = OverStepsActionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
