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
        # 座標系の設定を追加
        self.declare_parameter("global_frame", "map")  # 地図座標系 (map or odom)
        self.declare_parameter("robot_frame", "base_link")  # ロボット座標系

        self.declare_parameter("lookahead_dist", 0.4)
        self.declare_parameter("max_linear_vel", 0.3)
        self.declare_parameter("max_angular_vel", 1.0)
        self.declare_parameter("accel_limit", 0.5)
        self.declare_parameter("goal_tolerance", 0.05)
        self.declare_parameter("kp_linear", 1.5)
        self.declare_parameter("kp_yaw", 2.0)
        self.declare_parameter("control_frequency", 20.0)

        # --- TF & 状態管理 ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_path_index = 0
        self.last_vel = Twist()

        self.callback_group = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._action_server = ActionServer(
            self,
            FollowPath,
            'follow_path',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group)

        self.get_logger().info(
            f"Action Server started. Global: {self.get_parameter('global_frame').value}, Robot: {self.get_parameter('robot_frame').value}"
        )

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def get_robot_pose(self):
        # パラメータから座標系を取得
        global_f = self.get_parameter("global_frame").value
        robot_f = self.get_parameter("robot_frame").value

        try:
            # 指定された座標系間でルックアップ
            t = self.tf_buffer.lookup_transform(global_f, robot_f,
                                                rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception as e:
            # self.get_logger().warn(f"TF Wait: {e}")
            return None

    def find_lookahead(self, rx, ry, path):
        ld = self.get_parameter("lookahead_dist").value
        for i in range(self.current_path_index, len(path.poses)):
            px, py = path.poses[i].pose.position.x, path.poses[
                i].pose.position.y
            if math.hypot(px - rx, py - ry) >= ld:
                self.current_path_index = i
                return path.poses[i].pose
        return path.poses[-1].pose

    def get_quaternion_to_euler(self, q):
        return 0.0, 0.0, math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def clamp(self, val, min_value, max_value):
        return max(min_value, min(max_value, val))

    def publish_zero_vel(self):
        self.cmd_pub.publish(Twist())

    async def execute_callback(self, goal_handle):
        path = goal_handle.request.path
        self.current_path_index = 0
        self.last_vel = Twist()

        feedback_msg = FollowPath.Feedback()
        rate = self.create_rate(self.get_parameter("control_frequency").value)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.publish_zero_vel()
                return FollowPath.Result()

            pose = self.get_robot_pose()
            if pose is None:
                rate.sleep()
                continue
            rx, ry, ryaw = pose

            # ゴール判定
            gx, gy = path.poses[-1].pose.position.x, path.poses[
                -1].pose.position.y
            dist_to_goal = math.hypot(gx - rx, gy - ry)

            if dist_to_goal < self.get_parameter("goal_tolerance").value:
                self.publish_zero_vel()
                goal_handle.succeed()
                return FollowPath.Result()

            # Pure Pursuit
            lookahead_pt = self.find_lookahead(rx, ry, path)
            lx, ly = lookahead_pt.position.x, lookahead_pt.position.y

            # 座標変換 (Global -> Local)
            dx_map, dy_map = lx - rx, ly - ry
            vx_local = dx_map * math.cos(ryaw) + dy_map * math.sin(ryaw)
            vy_local = -dx_map * math.sin(ryaw) + dy_map * math.cos(ryaw)

            # 速度生成
            cmd = Twist()
            kp = self.get_parameter("kp_linear").value
            max_l = self.get_parameter("max_linear_vel").value

            raw_vx, raw_vy = vx_local * kp, vy_local * kp
            speed = math.hypot(raw_vx, raw_vy)
            if speed > max_l:
                raw_vx, raw_vy = (raw_vx / speed) * max_l, (raw_vy /
                                                            speed) * max_l

            # 加速度制限
            dt = 1.0 / self.get_parameter("control_frequency").value
            accel_lim = self.get_parameter("accel_limit").value * dt
            cmd.linear.x = self.clamp(raw_vx,
                                      self.last_vel.linear.x - accel_lim,
                                      self.last_vel.linear.x + accel_lim)
            cmd.linear.y = self.clamp(raw_vy,
                                      self.last_vel.linear.y - accel_lim,
                                      self.last_vel.linear.y + accel_lim)

            # 旋回制御
            _, _, gyaw = self.get_quaternion_to_euler(
                path.poses[-1].pose.orientation)
            yaw_err = math.atan2(math.sin(gyaw - ryaw), math.cos(gyaw - ryaw))
            cmd.angular.z = self.clamp(
                yaw_err * self.get_parameter("kp_yaw").value,
                -self.get_parameter("max_angular_vel").value,
                self.get_parameter("max_angular_vel").value)

            self.cmd_pub.publish(cmd)
            self.last_vel = cmd
            feedback_msg.distance_to_goal = dist_to_goal
            goal_handle.publish_feedback(feedback_msg)

            rate.sleep()


def main(args=None):
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
