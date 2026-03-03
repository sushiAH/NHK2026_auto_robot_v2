"""
ボックス持ち上げ
　　アームを下げる＆吸引ポンプオン＆角度調整
    時間が経過したら伸ばす
    時間が経過したら縮む
    時間が経過したらアームを上げる
ボックス置く
    ボックスを持った状態から
    アームを下げる
    時間が経過したら伸ばす
    吸引ポンプオフ＆縮む＆アームを上げる
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
from auto_robot_interfaces_v2.action import BoxArm
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
class BoxArmController(Node):

    def __init__(self):
        super().__init__("box_arm_controller")

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            BoxArm,
            "box_arm",
            self.execute_callback,
            callback_group=self.cb_group,
        )
        # publisherの設定
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        send_packet_1byte(0x020, 0, 4, CAN_BUS)  # set_operating
        send_packet_4byte(0x020, 3, 0, CAN_BUS)

        self.publish_dyna_pos(8, 1500)

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = BoxArm.Result()
        success = False

        #box置く
        if req.mode == 1:
            success = await self.put_box()

        #box持ち上げ
        elif req.mode == 2:
            success = await self.lift_box()

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    async def put_box(self):
        self.get_logger().info("put動作を開始します")

        #アームを下げる
        self.publish_dyna_pos(3, 2800)
        time.sleep(4.0)

        #時間が経過したら伸ばす
        self.publish_dyna_pos(2, 100)
        time.sleep(2.0)

        #吸引ポンプオフ＆縮む＆アームを上げる
        self.publish_dyna_pos(2, 3500)
        self.publish_dyna_pos(3, 1500)
        send_packet_4byte(0x020, 3, 0, CAN_BUS)
        time.sleep(2.0)

        self.get_logger().info("動作終了、待機します")
        return True

    async def lift_box(self):
        #アームを下げる＆先端＆吸引オン
        self.get_logger().info("lift動作を開始します")

        self.publish_dyna_pos(3, 2800)
        self.publish_dyna_pos(8, 1600)
        send_packet_4byte(0x020, 3, 1000, CAN_BUS)
        time.sleep(4.0)

        #伸ばす
        self.publish_dyna_pos(2, 100)
        time.sleep(2.0)

        #縮む
        self.publish_dyna_pos(2, 3500)
        time.sleep(2.0)

        #アームを上げる
        self.publish_dyna_pos(3, 1500)
        time.sleep(2.0)

        self.get_logger().info("動作終了、待機します")
        return True


def main():
    rclpy.init()
    node = BoxArmController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
