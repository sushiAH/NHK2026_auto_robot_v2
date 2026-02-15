import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from auto_robot_interfaces.action import PoseCorrection
import time
import math

#自作ライブラリ
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *


class PoseCorrectionServer(Node):

    def __init__(self):
        super().__init__("pose_correction_server")

        self.__action_server = ActionServer(self, PoseCorrection,
                                            "corrent_pose",
                                            self.execute_callback)

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info("自己位置を補正します")

        #初期化座標の取得
        msg = PoseWithCovarianceStamped()
        x, y, yaw = calc_correct_pos()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header_frame_id = "odom"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        #座標の送信
        self.inital_pose_pub.publish(msg)
        time.sleep(1.0)
        goal_handle.succeed()
        return PoseCorrection.Result(success=True)


def main():
    rclpy.init()
    node = PoseCorrectionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()
