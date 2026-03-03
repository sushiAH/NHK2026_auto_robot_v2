import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from auto_robot_interfaces.action import PoseCorrection
import time
import math
import asyncio

#自作ライブラリ
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from opencv_hough import *


class PoseCorrectionServer(Node):

    def __init__(self):
        super().__init__("pose_correction_server")
        self.cb_group = ReentrantCallbackGroup()

        self.__action_server = ActionServer(
            self,
            PoseCorrection,
            "correct_pose",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/set_pose",
            10,
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info("自己位置を補正します")

        #初期化座標の取得
        msg = PoseWithCovarianceStamped()
        x, y, yaw = calc_correct_pos()

        self.get_logger().info(f'x補正座標[mm]は {x} です')
        self.get_logger().info(f'y補正座標[mm]は {y} です')
        self.get_logger().info(f'yaw補正degreeは {yaw} です')

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        yaw = math.radians(yaw)

        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        #座標の送信
        self.initial_pose_pub.publish(msg)
        time.sleep(1.0)
        self.get_logger().info("動作終了、待機します")
        goal_handle.succeed()

        return PoseCorrection.Result(success=True)


def main():
    rclpy.init()
    node = PoseCorrectionServer()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
