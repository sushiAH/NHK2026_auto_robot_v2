"""
やり制御フロー
経路追従でやりの前まで来る
amclを無効化する
フレームを浮かす&やりハンドを正面に
odomを用いて前進する
取得する
odomを用いて後退する
フレームを下げる
開始位置を指定してamclを有効化する
arucoの待機


このプログラムには、
フレームを浮かす
odomを用いて前進する
取得する
odomを用いて後退する
までを書く
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

#自作ライブラリ
from auto_robot_interfaces_v2.action import Spear
from dyna_interfaces.msg import DynaFeedback, DynaTarget
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *


def calc_frame_height(dis_cm):
    return int((360 / (45 * math.pi) / 0.088) * (dis_cm))


# ----Config Params -----
class SpearController(Node):

    def __init__(self):
        super().__init__("spear_controller")

        self._action_server = ActionServer(
            self,
            Spear,
            "spear",
            self.execute_callback,
        )

        # publisherの設定
        self.dyna_extpos_publisher = self.create_publisher(
            DynaTarget, "/dyna_target_extpos", 10)

        self.dyna_vel_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_vel", 10)

        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        #----Params----
        self.spear_arm_pos_list = [1674, 2660]  #上、下
        self.spear_hand_pos_list = [1440, 350]  #開く、閉じる
        self.spear_frame_height = 210

        #やりハンド初期化
        self.publish_dyna_pos(10, self.spear_arm_pos_list[0])
        self.publish_dyna_pos(11, self.spear_hand_pos_list[0])

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

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def publish_dyna_twist(self, vx):
        self.publish_dyna_vel(0, -vx)
        self.publish_dyna_vel(1, vx)

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = Spear.Result()
        success = False
        success = await self.spaer()

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    async def spear(self):
        self.get_logger().info("やり取得動作を開始します。")

        #フレームを上げる&やりハンドを正面に
        self.publish_dyna_extpos(2, calc_frame_height(-spear_frame_height))
        self.publish_dyna_extpos(3, calc_frame_height(spear_frame_height))
        self.publish_dyna_pos(10, self.spear_arm_pos_list[1])  #下
        self.publish_dyna_pos(11, self.spear_hand_pos_list[0])  #開
        time.sleep(4.0)

        #一定時間前に進む
        self.publish_dyna_twist(100)
        time.sleep(1.0)

        #やりハンドで掴む
        self.publish_dyna_pos(11, self.spear_hand_pos_list[1])  #閉
        time.sleep(1.0)

        #やり回転を上向きに
        self.publish_dyna_pos(10, self.spear_arm_pos_list[0])  #上
        time.sleep(1.0)

        #一定時間後ろに進む
        self.publish_dyna_twist(-100)
        time.sleep(1.0)

        #フレームを下げる＆ハンド開く
        self.publish_dyna_extpos(2, calc_frame_height(10))
        self.publish_dyna_extpos(3, calc_frame_height(-10))
        self.publish_dyna_pos(11, self.spear_arm_pos_list[0])  #開

        self.get_logger().info("やり取得動作を終了しました。待機します。")


def main(args=None):
    rclpy.init(args=args)
    node = SpearController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
