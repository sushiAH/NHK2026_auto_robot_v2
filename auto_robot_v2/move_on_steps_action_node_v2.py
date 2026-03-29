"""
段上での制御フロー
mode 
    1: start->center 1
    2: center->left 2
    3: center->right 3
    4: center->straight 4
    5: left->center 5
    6: right->center 6
    7: straight->center 7
    8: center->steps 8

"""

import os
import csv
import rclpy
import math
import numpy as np
import time

from std_msgs.msg import UInt8, UInt16, String
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from tf2_ros import TransformBroadcaster

from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import FollowPath
import atexit

import asyncio
from rclpy.action import ActionServer, CancelResponse, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from action_msgs.msg import GoalStatus

#自作ライブラリ
from auto_robot_interfaces_v2.action import MoveOnSteps
from dyna_interfaces.msg import DynaFeedback, DynaTarget
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *

# ----Config Params -----


def quaternion_to_yaw(x, y, w, z):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw


def limit_value(value, limit_value_abs):
    if (value > limit_value_abs):
        return limit_value_abs
    elif (value < -limit_value_abs):
        return -limit_value_abs

    return value


def calc_p(target, current, p_gain):
    p_value = (target - current) * p_gain
    p_value = limit_value(p_value, 0.5)

    return p_value


def is_in_threshold(target, current, threshold):
    e = target - current
    if (abs(e) < threshold):
        return True

    return False


class OnStepsController(Node):

    def __init__(self):
        super().__init__("on_steps_controller")
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            MoveOnSteps,
            "move_on_steps",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        self._action_client_path = ActionClient(self,
                                                FollowPath,
                                                "follow_path_odom",
                                                callback_group=self.cb_group)

        # publisherの設定
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        #twistの配信
        self.twist_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        #odomの受取
        self.subscription = self.create_subscription(
            Odometry,
            "/odometry/filtered",
            self.odom_callback,
            10,
            callback_group=self.cb_group,
        )

        self.x = 0.00
        self.y = 0.00
        self.yaw = 0.00

        # ----Params----
        self.start_to_center = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/start_to_center.csv"
        )
        self.center_to_left = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/center_to_left.csv"
        )
        self.center_to_right = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/center_to_right.csv"
        )

        self.center_to_straight = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/center_to_straight.csv"
        )

        self.left_to_center = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/left_to_center.csv"
        )
        self.right_to_center = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/right_to_center.csv"
        )
        self.straight_to_center = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/straight_to_center.csv"
        )

        self.center_to_steps = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/center_to_steps.csv"
        )

    def load_path_from_csv(self, file_path):
        """
        CSVファイルを読み込んで nav_msgs/msg/Path を作成する
        CSV形式: x, y, yaw (1行目はヘッダーを想定)
        """
        path_msg = Path()
        # タイムスタンプは送信直前に設定するのが一般的
        path_msg.header.frame_id = "odom"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        if not os.path.exists(file_path):
            print(f"Error: File not found {file_path}")
            return None

        with open(file_path, 'r') as f:
            reader = csv.reader(f)
            header = next(reader)  # ヘッダーをスキップ

            for row in reader:
                if not row:
                    continue

                # 文字列を数値に変換
                x, y, yaw = map(float, row)

                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x
                pose.pose.position.y = y

                # Yaw角をクォータニオンに変換
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)

                path_msg.poses.append(pose)

        return path_msg

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.w, q.z)

        self.x = pos.x
        self.y = pos.y
        self.yaw = math.degrees(yaw)

    def publish_twist(self, vx, vy, w):
        twist = Twist()
        twist.linear.x = float(vx)
        twist.linear.y = float(vy)
        twist.angular.z = float(w)
        self.twist_publisher.publish(twist)

    # --- Action実行用の共通内部メソッド ---
    async def _send_action_goal(self, client, goal_msg, action_name):
        """Actionを送信し、完了(SUCCEEDED)まで待機する共通処理"""
        self.get_logger().info(f"[{action_name}] サーバーを待機中...")
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"[{action_name}] サーバーが見つかりません")
            return False

        self.get_logger().info(f"[{action_name}] ゴール送信開始")
        send_goal_future = await client.send_goal_async(goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().error(f"[{action_name}] 命令が拒否されました")
            return False

        self.get_logger().info(f"[{action_name}] 受理されました。結果を待機します...")
        result_handle = await send_goal_future.get_result_async()

        # ステータスコードのチェック
        if result_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[{action_name}] 正常完了しました")
            return result_handle.result
        else:
            self.get_logger().warn(
                f"[{action_name}] 失敗しました (Status ID: {result_handle.status})")
            return None

    async def send_goal(self, path_msg):
        if path_msg is None:
            return False
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        return await self._send_action_goal(self._action_client_path, goal_msg,
                                            "FollowPath")

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = MoveOnSteps.Result()
        success = False

        #start->right
        if req.mode == 1:
            success = await self.follow_path(self.start_to_center)

        #start->left
        elif req.mode == 2:
            success = await self.follow_path(self.center_to_left)

        #start->straight
        elif req.mode == 3:
            success = await self.follow_path(self.center_to_right)

        #right->straight
        elif req.mode == 4:
            success = await self.follow_path(self.center_to_straight)

        #left->straight
        elif req.mode == 5:
            success = await self.follow_path(self.left_to_center)

        #stright->goal
        elif req.mode == 6:
            success = await self.follow_path(self.right_to_center)

        elif req.mode == 7:
            success = await self.follow_path(self.straight_to_center)

        elif req.mode == 8:
            success = await self.follow_path(self.center_to_steps)

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    async def follow_path(self, path_msg):

        self.get_logger().info("段上動作")
        await self.send_goal(path_msg)
        self.get_logger().info("動作終了、待機します")

        return True


def main(args=None):
    rclpy.init(args=args)
    node = OnStepsController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
