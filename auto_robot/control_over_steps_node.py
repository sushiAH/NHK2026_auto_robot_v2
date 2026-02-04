"""
段差超え機構
---------制御物-----------
足回りdynamixel 2つ 速度制御
フレーム昇降 dynamixel 2つ 拡張位置制御
tofセンサー2つ
まぶちモーター4つ

---------nodeの仕様-------------
----publisher----
Twist(足回り)
current_status
dyna_extpos

----subscriber----
target_status
tof_forward
tof_backward

---------段差超え/制御フロー------------
最初は(600,600)の状態
フレーム上げる(210,210) [mm]
前方に進める
tof_forwardの値が小さくなったら(段差が近づいたら)、車体を止めて、前方の足を上げる(-10,210) [cm]
決まった秒数が立ったら、dcモーターと、dynamixelを前方に動かす
tof_backwardの値が小さくなったら(段差が近づいたら)、車体を止めて、後方の足を上げる(-10,-10) [cm]
dcモーターと、dynamixelを前方に動かす
決まった秒数が立ったら終了
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
from my_robot_interfaces.msg import DynaFeedback
import atexit

#自作系
from my_robot_interfaces.msg import DynaFeedback, DynaTarget
from auto_robot.lib.ah_python_can import *

#initialize_can_bus
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


class OverStepsController(Node):

    def __init__(self):
        super().__init__("over_steps_controller")
        #subscriberの設定
        self.subscription_status = self.create_subscription(
            UInt8, "/over_steps_target_status", self.subscribe_status, 10)
        self.subscription_status

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

        #current_statusの配信(段差幟町か、段差降りまちか)
        #ここのstatusで、段差上にいるか、段差下にいるか判断する予定
        self.current_status_publisher = self.create_publisher(
            UInt8, "/over_steps_current_status", 10)

        # timer callback の周期設定
        self.dt = 0.1  # [seconds] [0.1]
        self.timer = self.create_timer(self.dt, self.control_over_steps)

        #内部変数の定義
        self.tof_forward = 0
        self.tof_backward = 0
        self.target_status = 0
        self.current_status = 0
        self.inner_status = 0
        self.past_time = time.perf_counter()

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

    def over_steps(self):
        now_time = time.perf_counter()
        diff_time = now_time - self.past_time

        #twistのpublishをする
        if self.inner_status == 0:
            print("指示を受け取りました、動作を開始します")
            print("フレームを上昇します")
            self.publish_dyna_extpos(4, calc_frame_height(230))
            self.publish_dyna_extpos(5, calc_frame_height(230))

            self.inner_status = 1
            self.past_time = now_time
        elif self.inner_status == 1 and diff_time > 4.00:
            print("直進します")
            self.publish_twist(0.1, 0)

            self.inner_status = 2
            self.past_time = now_time
        elif self.inner_status == 2 and self.tof_forward < TOF_THRESHOLD:
            print("段差を検知しました、車体を停止し,前方を上昇します")
            self.publish_twist(0, 0)
            move_motor(0)
            self.publish_dyna_extpos(4, calc_frame_height(-10))

            self.inner_status = 3
            self.past_time = now_time
        elif self.inner_status == 3 and diff_time > 4.00:
            print("直進します")
            self.publish_twist(0.1, 0)
            move_motor(400)

            self.inner_status = 4
            self.past_time = now_time
        elif self.inner_status == 4 and self.tof_backward < TOF_THRESHOLD:
            print("段差を検知しました、車体を停止し、後方の足を上昇します")
            self.publish_twist(0, 0)
            move_motor(0)
            self.publish_dyna_extpos(5, calc_frame_height(-10))

            self.inner_status = 5
            self.past_time = now_time
        elif self.inner_status == 5 and diff_time > 4.00:
            print("直進します")
            self.publish_twist(0.1, 0)
            move_motor(400)

            self.inner_status = 6
            self.past_time = now_time
        elif self.inner_status == 6 and diff_time > 2.00:
            print("車体を停止します。段差超えを完了しました")
            self.publish_twist(0, 0)
            move_motor(0)

            self.inner_status = 0
            self.current_status = 1

            #current_statusの配信
            current_status_msg = UInt8()
            current_status_msg.data = 1
            self.current_status_publisher.publish(current_status_msg)
            self.past_time = now_time

        print("diff_time", diff_time)
        print("inner_status", self.inner_status)

    def subscribe_status(self, msg):
        self.target_status = msg.data

    def subscribe_tof_forward(self, msg):
        self.tof_forward = msg.data

    def subscribe_tof_backward(self, msg):
        self.tof_backward = msg.data

    def control_over_steps(self):
        if (self.current_status == 0 and self.target_status == 1):
            self.over_steps()

        elif (self.current_status == 1 and self.target_status == 2):
            pass


def main():
    rclpy.init()  # rclpyライブラリの初期化
    over_steps_node = OverStepsController()
    rclpy.spin(over_steps_node)  # ノードをスピンさせる
    over_steps_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
