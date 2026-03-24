#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath

from tf2_ros import Buffer, TransformListener


class OmniPurePursuitActionServer(Node):

    def __init__(self):
        super().__init__("omni_pure_pursuit_action_server")

        # --- パラメータ設定 (YAMLや起動引数から変更可能) ---
        # 座標系の設定を追加
        self.declare_parameter("global_frame", "map")  # 基準座標系 (map / odom)
        self.declare_parameter("robot_frame", "base_link")  # ロボット座標系

        self.declare_parameter("max_linear_vel", 1.0)  # 最高速度 [m/s]
        self.declare_parameter("accel_limit", 1.5)  # 加速度制限 [m/s^2]
        self.declare_parameter("min_lookahead_dist", 0.3)  # 最小注視距離 [m]
        self.declare_parameter("lookahead_gain", 0.5)  # 注視距離ゲイン
        self.declare_parameter("max_angular_vel", 1.5)  # 最大旋回速度 [rad/s]
        self.declare_parameter("kp_yaw", 3.0)  # 旋回Pゲイン
        self.declare_parameter("goal_tolerance", 0.05)  # 到着許容誤差 [m]

        # --- TF & 通信 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # アクションサーバー設定
        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        # 設定の確認用ログ
        g_frame = self.get_parameter("global_frame").value
        r_frame = self.get_parameter("robot_frame").value
        self.get_logger().info(
            f"Ready for NHK Robocon 2026! Frame: {g_frame} -> {r_frame}")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def clamp(self, val, min_value, max_value):
        return max(min_value, min(max_value, val))

    def get_quaternion_to_euler(self, q):
        return 0.0, 0.0, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def publish_zero_vel(self):
        self.cmd_pub.publish(Twist())

    # --- 1. 速度計画関数 ---
    def plan_velocity_profile(self, path):
        max_v = self.get_parameter("max_linear_vel").value
        accel = self.get_parameter("accel_limit").value

        num_points = len(path.poses)
        distances = [0.0] * num_points
        total_dist = 0.0

        for i in range(1, num_points):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            total_dist += math.hypot(p2.x - p1.x, p2.y - p1.y)
            distances[i] = total_dist

        velocity_profile = []
        for s in distances:
            v_accel = math.sqrt(2 * accel * s)
            v_decel = math.sqrt(2 * accel * max(0.0, total_dist - s))
            target_v = max(0.1, min(v_accel, max_v, v_decel))

            t = Twist()
            t.linear.x = target_v
            velocity_profile.append(t)

        return velocity_profile, total_dist

    # --- 3. サポートロジック ---
    def get_robot_pose(self):
        # パラメータからフレーム名を取得してTFを参照
        g_frame = self.get_parameter("global_frame").value
        r_frame = self.get_parameter("robot_frame").value
        try:
            t = self.tf_buffer.lookup_transform(g_frame, r_frame,
                                                rclpy.time.Time())
            q = t.transform.rotation
            _, _, yaw = self.get_quaternion_to_euler(q)
            return t.transform.translation.x, t.transform.translation.y, yaw
        except:
            return None

    def get_closest_index(self, rx, ry, path, start_idx):
        min_d = float('inf')
        closest = start_idx
        for i in range(start_idx, min(start_idx + 20, len(path.poses))):
            d = math.hypot(path.poses[i].pose.position.x - rx,
                           path.poses[i].pose.position.y - ry)
            if d < min_d:
                min_d = d
                closest = i
        return closest

    def find_lookahead_point(self, rx, ry, path, start_idx, ld):
        for i in range(start_idx, len(path.poses)):
            if math.hypot(path.poses[i].pose.position.x - rx,
                          path.poses[i].pose.position.y - ry) >= ld:
                return path.poses[i].pose.position
        return path.poses[-1].pose.position

    def control_step(self, rx, ry, ryaw, target_pt, v_cmd, goal_quat):
        dx, dy = target_pt.x - rx, target_pt.y - ry
        angle_to_target = math.atan2(dy, dx) - ryaw

        cmd = Twist()
        cmd.linear.x = v_cmd * math.cos(angle_to_target)
        cmd.linear.y = v_cmd * math.sin(angle_to_target)

        _, _, gyaw = self.get_quaternion_to_euler(goal_quat)
        yaw_err = math.atan2(math.sin(gyaw - ryaw), math.cos(gyaw - ryaw))
        cmd.angular.z = max(
            -1.0, min(1.0,
                      yaw_err * self.get_parameter("kp_yaw").value))

        self.cmd_pub.publish(cmd)

    # --- 2. メイン実行ループ ---
    async def execute_callback(self, goal_handle):
        path = goal_handle.request.path
        if not path.poses:
            goal_handle.abort()
            return FollowPath.Result()

        v_profile, total_path_len = self.plan_velocity_profile(path)

        current_idx = 0
        rate = self.create_rate(20)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_zero_vel()
                return FollowPath.Result()

            pose = self.get_robot_pose()
            if not pose:
                rate.sleep()
                continue
            rx, ry, ryaw = pose

            current_idx = self.get_closest_index(rx, ry, path, current_idx)

            v_planned = v_profile[current_idx].linear.x
            ld = self.get_parameter("min_lookahead_dist").value + \
                 (self.get_parameter("lookahead_gain").value * v_planned)

            dist_to_goal = math.hypot(path.poses[-1].pose.position.x - rx,
                                      path.poses[-1].pose.position.y - ry)
            if dist_to_goal < self.get_parameter("goal_tolerance").value:
                break

            lookahead_pt = self.find_lookahead_point(rx, ry, path, current_idx,
                                                     ld)
            self.control_step(rx, ry, ryaw, lookahead_pt, v_planned,
                              path.poses[-1].pose.orientation)

            rate.sleep()

        self.publish_zero_vel()
        goal_handle.succeed()
        return FollowPath.Result()


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = OmniPurePursuitActionServer()
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
