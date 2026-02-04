"""
ボックス回収アームハンドを制御する
状態の指示を受け取る
dynamixelにpublishを行う
"""

import rclpy
import math
import numpy as np
import time

from my_robot_interfaces.msg import DynaFeedback, DynaTarget
from std_msgs.msg import UInt8, String
from rclpy.node import Node


class BoxArmController(Node):

    def __init__(self):
        super().__init__("box_arm_controller")
        self.subscription_status = self.create_subscription(
            UInt8, "/box_arm_status", self.subscribe_status, 10)
        self.subscription_status

        # publisherの設定
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        # timer callback の周期設定
        self.dt = 0.1  # [seconds] [0.1]
        self.timer = self.create_timer(self.dt, self.control_box_arm)

        #内部変数の定義
        self.target_status = 0
        self.current_status = 0
        self.inner_status = 0
        self.past_time = time.perf_counter()

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    def pick_up(self):

        now_time = time.perf_counter()
        diff_time = now_time - self.past_time

        # 回転
        if self.inner_status == 0:
            print("指示を受け取りました、動作を開始します")
            self.publish_dyna_pos(2, 300)
            self.publish_dyna_pos(3, 800)
            self.inner_status = 1
            self.past_time = now_time

        # 伸びる
        elif self.inner_status == 1 and diff_time > 4.00:
            self.publish_dyna_pos(2, 300)
            self.publish_dyna_pos(3, 3700)
            self.inner_status = 2
            self.past_time = now_time

        # 縮む
        elif self.inner_status == 2 and diff_time > 2.00:
            self.publish_dyna_pos(2, 300)
            self.publish_dyna_pos(3, 800)
            self.inner_status = 3
            self.past_time = now_time

        # 回転
        elif self.inner_status == 3 and diff_time > 2.00:
            self.publish_dyna_pos(2, 1500)
            self.publish_dyna_pos(3, 800)

            self.inner_status = 0  #初期化
            self.current_status = 1
            self.past_time = now_time
            print("動作を終了します")

        print("diff_time", diff_time)
        print("inner_status", self.inner_status)

    def subscribe_status(self, msg):
        self.target_status = msg.data

    def control_box_arm(self):
        if (self.current_status == 0 and self.target_status == 1):
            self.pick_up()

        elif (self.current_status == 1 and self.target_status == 2):
            pass


def main():
    rclpy.init()  # rclpyライブラリの初期化
    box_arm_controller_node = BoxArmController()
    rclpy.spin(box_arm_controller_node)  # ノードをスピンさせる
    box_arm_controller_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
