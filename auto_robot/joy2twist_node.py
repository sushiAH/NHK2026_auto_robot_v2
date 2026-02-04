"""joyを受け取って、twistをpubilshする
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

from my_robot_interfaces.msg import DynaFeedback


class Joy2Twist(Node):

    def __init__(self):
        super().__init__("joy2twist")
        self.subscription_joy = self.create_subscription(
            Joy,  # メッセージの型
            "/joy",  # 購読するトピック名
            self.joy_callback,  # 呼び出すコールバック関数
            10,
        )
        self.subscription_joy

        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # -----robot parameter-----
        self.straight_gain = 0.2
        self.omega_gain = 0.5

    def joy_callback(self, msg):

        axes_values = msg.axes
        buttons_values = msg.buttons

        straight = axes_values[1] * self.straight_gain
        w = axes_values[0] * self.omega_gain

        twist = Twist()
        twist.linear.x = straight
        twist.angular.z = w

        self.twist_publisher.publish(twist)


def main():
    rclpy.init()  # rclpyライブラリの初期化

    joy2twist_node = Joy2Twist()

    rclpy.spin(joy2twist_node)  # ノードをスピンさせる
    joy2twist_node.destroy_node()  # ノードを停止する
    rclpy.shutdown()


if __name__ == "__main__":
    main()
