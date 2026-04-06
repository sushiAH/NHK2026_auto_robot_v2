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

        # --- 1. 基本走行パラメータ ---
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("max_linear_vel", 2.0)
        self.declare_parameter("accel_limit", 2.5)  # 加速度（短い経路の立ち上がりに影響）
        self.declare_parameter("min_lookahead_dist", 0.2)
        self.declare_parameter("lookahead_gain", 0.2)
        self.declare_parameter("max_angular_vel", 1.5)
        self.declare_parameter("kp_yaw", 1.5)

        # --- 2. 到着判定・減速制御パラメータ ---
        self.declare_parameter("goal_tolerance", 0.05)  # 到着判定（距離 [m]）
        self.declare_parameter("yaw_tolerance", 0.05)  # 到着判定（角度 [rad]）

        self.declare_parameter("v_start_min", 0.2)  # 摩擦に負けない最小速度
        self.declare_parameter("decel_start_dist", 0.4)  # 計画上の減速開始距離

        # 【核心】最低速度保証を維持する距離
        # ゴールまでこの距離を切るまでは、v_start_min を下回らないように強制する
        self.declare_parameter("min_vel_enforce_dist", 0.15)

        self.declare_parameter("action_name", "follow_path_map")
        action_name = self.get_parameter("action_name").value

        # --- 3. 通信・TF設定 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Action Server の設定 (ReentrantCallbackGroup で並列処理を許可)
        self._action_server = ActionServer(
            self,
            FollowPath,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f"NHK2026 Pure Pursuit Server Ready: [{action_name}]")

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def get_quaternion_to_euler(self, q):
        # 四元数から Yaw 角のみを算出
        return 0.0, 0.0, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def publish_zero_vel(self):
        self.cmd_pub.publish(Twist())

    def plan_velocity_profile(self, path):
        # 経路全体の累積距離に基づいた速度プロファイル（台形加速）を作成
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
            # 加速、最高速、減速の最小値を採用
            velocity_profile.append(min(v_accel, max_v, v_decel))
        return velocity_profile

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

    def control_step(self, rx, ry, ryaw, target_pt, v_cmd, goal_quat):
        # 平行移動の計算
        dx, dy = target_pt.x - rx, target_pt.y - ry
        angle_to_target = math.atan2(dy, dx) - ryaw

        cmd = Twist()
        cmd.linear.x = v_cmd * math.cos(angle_to_target)
        cmd.linear.y = v_cmd * math.sin(angle_to_target)

        # 旋回（姿勢角）の計算
        _, _, gyaw = self.get_quaternion_to_euler(goal_quat)
        yaw_err = math.atan2(math.sin(gyaw - ryaw), math.cos(gyaw - ryaw))

        max_ang = self.get_parameter("max_angular_vel").value
        kp_y = self.get_parameter("kp_yaw").value
        z_out = yaw_err * kp_y

        # 旋回不感帯対策：誤差があるなら最低限回るパワーを出す
        min_z = 0.1
        if abs(yaw_err) > self.get_parameter("yaw_tolerance").value:
            if abs(z_out) < min_z:
                z_out = math.copysign(min_z, z_out)

        cmd.angular.z = max(-max_ang, min(max_ang, z_out))
        self.cmd_pub.publish(cmd)

    async def execute_callback(self, goal_handle):
        path = goal_handle.request.path
        if not path.poses:
            goal_handle.abort()
            return FollowPath.Result()

        self.get_logger().info("追従開始")
        v_profile = self.plan_velocity_profile(path)
        current_idx = 0

        # 制御ループの周期設定 (20Hz)
        rate = self.create_rate(20)

        while rclpy.ok():
            # 1. キャンセル確認
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_zero_vel()
                return FollowPath.Result()

            # 2. ロボットの現在地取得
            pose = self.get_robot_pose()
            if not pose:
                rate.sleep()
                continue

            rx, ry, ryaw = pose

            # 3. 最近傍点の検索
            min_d = float('inf')
            search_range = min(current_idx + 50, len(path.poses))
            for i in range(current_idx, search_range):
                d = math.hypot(path.poses[i].pose.position.x - rx,
                               path.poses[i].pose.position.y - ry)
                if d < min_d:
                    min_d = d
                    current_idx = i

            # 4. 誤差の計算
            dist_to_goal = math.hypot(path.poses[-1].pose.position.x - rx,
                                      path.poses[-1].pose.position.y - ry)
            _, _, gyaw = self.get_quaternion_to_euler(
                path.poses[-1].pose.orientation)
            yaw_err = math.atan2(math.sin(gyaw - ryaw), math.cos(gyaw - ryaw))

            # 5. 終了判定
            if dist_to_goal < self.get_parameter("goal_tolerance").value and \
               abs(yaw_err) < self.get_parameter("yaw_tolerance").value:
                break

            # 6. 速度決定（今回のスタック対策ロジック）
            v_planned = v_profile[current_idx]

            # 指定された「最低速度保証距離」より遠ければ v_start_min を下限にする
            if dist_to_goal > self.get_parameter("min_vel_enforce_dist").value:
                v_final = max(v_planned,
                              self.get_parameter("v_start_min").value)
            else:
                # ゴール目前なら、滑らかに停止させるために計算値を優先
                v_final = v_planned

            # 7. Lookahead Point（注視点）の決定
            ld = self.get_parameter("min_lookahead_dist").value + \
                 (self.get_parameter("lookahead_gain").value * v_final)

            target_pt = path.poses[-1].pose.position
            for i in range(current_idx, len(path.poses)):
                if math.hypot(path.poses[i].pose.position.x - rx,
                              path.poses[i].pose.position.y - ry) >= ld:
                    target_pt = path.poses[i].pose.position
                    break

            # 8. 制御指令のパブリッシュ
            self.control_step(rx, ry, ryaw, target_pt, v_final,
                              path.poses[-1].pose.orientation)

            # ログ出力（デバッグ用）
            self.get_logger().info(
                f"Dist: {dist_to_goal:.3f}, V: {v_final:.2f}",
                throttle_duration_sec=0.5)

            # 待機
            rate.sleep()

        # 9. 停止処理とアクション終了
        self.publish_zero_vel()
        self.get_logger().info("目標地点に到達しました。")
        goal_handle.succeed()
        return FollowPath.Result()


def main(args=None):
    rclpy.init(args=args)
    # マルチスレッド実行を許可（TF更新などの並行処理のため）
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
