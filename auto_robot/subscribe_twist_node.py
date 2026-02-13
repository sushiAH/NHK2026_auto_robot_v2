"""twistを受け取って、dynamixelモーターに指示値を送信する。
/dyna_target_posトピックを送信することで、dynamixelモーターに指示値を送信する
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import math
import numpy as np
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import atexit

from auto_robot_interfaces.msg import DynaFeedback, DynaTarget


class TwistSubscriber(Node):

    def __init__(self):
        super().__init__("twist_subscriber")

        self.dyna_vel_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_vel", 10)
        self.subscription_twist = self.create_subscription(
            Twist,  # メッセージの型
            "/cmd_vel",  # 購読するトピック名
            self.twist2dyna,  # 呼び出すコールバック関数
            10,
        )
        self.subscription_twist

        # robot_params
        self.track_width = 0.285  # [m]
        self.wheel_radius = 0.041  # [m]
        self.dyna_vel_gain = (0.229 * 2.0 * math.pi * self.wheel_radius) / 60.0

    def publish_dyna_vel(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_vel_publisher.publish(msg)

    def twist2dyna(self, msg):

        straight = -msg.linear.x
        w = msg.angular.z

        V_r = int(
            (2 * straight - w * self.track_width) / (2 * self.dyna_vel_gain))
        V_l = -int(
            (2 * straight + w * self.track_width) / (2 * self.dyna_vel_gain))

        self.publish_dyna_vel(0, V_r)  #差動二輪の右車輪
        self.publish_dyna_vel(1, V_l)  #差動二輪の左車輪


def main():
    rclpy.init()  # rclpyライブラリの初期化

    twist_subscriber_node = TwistSubscriber()

    rclpy.spin(twist_subscriber_node)  # ノードをスピンさせる
    twist_subscriber_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
