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

        # --- パラメータ設定 ---
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")

        # 物理限界（まずは安全な2.0程度から。現場で追い込んでください）
        self.declare_parameter("max_linear_vel", 2.0)
        self.declare_parameter("accel_limit", 2.0)

        # Pure Pursuit用
        self.declare_parameter("min_lookahead_dist", 0.2)
        self.declare_parameter("lookahead_gain", 0.2)

        # 旋回用
        self.declare_parameter("max_angular_vel", 1.5)
        self.declare_parameter("kp_yaw", 1.5)

        # 到着判定
        self.declare_parameter("goal_tolerance", 0.05)

        # 出だしブースト用の設定
        self.declare_parameter("v_start_min", 1.5)  # 摩擦に勝つための最小速度
        self.declare_parameter("decel_start_dist", 0.4)  # 減速（下限解除）を開始する距離

        # アクション名の取得
        self.declare_parameter("action_name", "follow_path_map")
        action_name = self.get_parameter("action_name").value

        self.path_pub = self.create_publisher(Path, "/visual_spline_path", 10)

        # --- TF & 通信 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._action_server = ActionServer(
            self,
            FollowPath,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f"NHK2026 Pure Pursuit Node: [{action_name}] is ready.")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def get_quaternion_to_euler(self, q):
        return 0.0, 0.0, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def publish_zero_vel(self):
        self.cmd_pub.publish(Twist())

    # --- 1. 純粋な台形速度計画 ---
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
            # 純粋な物理式に基づく台形加速（下限は実行時に処理する）
            v_accel = math.sqrt(2 * accel * s)
            v_decel = math.sqrt(2 * accel * max(0.0, total_dist - s))
            target_v = min(v_accel, max_v, v_decel)
            velocity_profile.append(target_v)

        return velocity_profile

    # --- 3. サポートロジック ---
    def get_robot_pose(self):
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
        # 全探索せず、現在位置の少し先までを効率よく探す
        search_range = min(start_idx + 50, len(path.poses))
        for i in range(start_idx, search_range):
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
        # 目標点への相対角度
        dx, dy = target_pt.x - rx, target_pt.y - ry
        angle_to_target = math.atan2(dy, dx) - ryaw

        cmd = Twist()
        # オムニ移動：進行方向に対して機体の向きを維持したまま進む
        cmd.linear.x = v_cmd * math.cos(angle_to_target)
        cmd.linear.y = v_cmd * math.sin(angle_to_target)

        # 旋回制御：ゴールの向きに合わせる
        _, _, gyaw = self.get_quaternion_to_euler(goal_quat)
        yaw_err = math.atan2(math.sin(gyaw - ryaw), math.cos(gyaw - ryaw))

        max_ang = self.get_parameter("max_angular_vel").value
        kp_y = self.get_parameter("kp_yaw").value
        cmd.angular.z = max(-max_ang, min(max_ang, yaw_err * kp_y))

        self.cmd_pub.publish(cmd)

    # --- 2. メイン実行ループ ---
    async def execute_callback(self, goal_handle):
        path = goal_handle.request.path
        if not path.poses:
            goal_handle.abort()
            return FollowPath.Result()

        self.get_logger().info("経路を受け取りました、追従を開始します")

        v_profile = self.plan_velocity_profile(path)
        current_idx = 0
        rate = self.create_rate(20)

        # パラメータ読み込み
        v_start_min = self.get_parameter("v_start_min").value
        decel_dist = self.get_parameter("decel_start_dist").value
        goal_tol = self.get_parameter("goal_tolerance").value

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

            # 最寄りの点を探す
            current_idx = self.get_closest_index(rx, ry, path, current_idx)

            # ゴールまでの距離を計算
            dist_to_goal = math.hypot(path.poses[-1].pose.position.x - rx,
                                      path.poses[-1].pose.position.y - ry)

            # 到着判定
            if dist_to_goal < goal_tol:
                break

            # --- 動的な速度制御 (Dynamic V-Min) ---
            v_planned = v_profile[current_idx]

            if dist_to_goal > decel_dist:
                # 出だし〜中間：摩擦に勝つために下限を設ける
                v_final = max(v_start_min, v_planned)
            else:
                # ゴール付近：下限を解除し、0へ向かってスムーズに減速
                v_final = v_planned

            # 注視距離の計算
            ld = self.get_parameter("min_lookahead_dist").value + \
                 (self.get_parameter("lookahead_gain").value * v_final)

            lookahead_pt = self.find_lookahead_point(rx, ry, path, current_idx,
                                                     ld)
            self.control_step(rx, ry, ryaw, lookahead_pt, v_final,
                              path.poses[-1].pose.orientation)

            rate.sleep()

        self.publish_zero_vel()
        self.get_logger().info("到着しました。待機します。")
        goal_handle.succeed()
        return FollowPath.Result()


def main(args=None):
    # Launchファイルからのリマップを受け取るために args=args が必須
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = OmniPurePursuitActionServer()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero_vel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
