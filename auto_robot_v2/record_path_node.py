#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
import csv
import os
from tf_transformations import euler_from_quaternion
from sensor_msgs.msg import Joy


class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')

        # ---- パラメータ ----
        # 座標系の設定を追加
        self.declare_parameter('global_frame', 'map')  # 基準座標系 (map / odom)
        self.declare_parameter('robot_frame', 'base_link')  # ロボット座標系

        self.declare_parameter(
            'directory', '~/NHK2026_auto_robot_v2/src/auto_robot_v2/path/')
        self.declare_parameter('dist_threshold', 0.01)

        # 保存ディレクトリの準備
        self.save_dir = os.path.expanduser(
            self.get_parameter('directory').value)
        os.makedirs(self.save_dir, exist_ok=True)

        self.dist_threshold = self.get_parameter('dist_threshold').value

        # ---- 変数 ----
        self.is_recording = False
        self.last_button_state = 0
        self.path_data = []
        self.last_pose = None
        self.file_count = 0

        # TF2 / Joy
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback,
                                                10)
        self.timer = self.create_timer(0.05, self.record_loop)

        g_frame = self.get_parameter('global_frame').value
        r_frame = self.get_parameter('robot_frame').value
        self.get_logger().info(f"Recorder Ready. Frame: {g_frame} -> {r_frame}")

    def joy_callback(self, msg):
        # OPTIONSボタン (PS4: buttons[9])
        if msg.buttons[9] == 1 and self.last_button_state == 0:
            if not self.is_recording:
                self.path_data = []
                self.last_pose = None
                self.is_recording = True
                self.get_logger().info(f"● 記録開始 (Path {self.file_count})")
            else:
                self.is_recording = False
                self.save_current_path()
                self.file_count += 1
                self.get_logger().info(f"■ 記録停止・保存完了")

        self.last_button_state = msg.buttons[9]

    def record_loop(self):
        if not self.is_recording:
            return

        # パラメータから現在の設定を取得
        g_frame = self.get_parameter('global_frame').value
        r_frame = self.get_parameter('robot_frame').value

        try:
            # ハードコードされていた 'map', 'base_link' を変数に変更
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(g_frame, r_frame, now)

            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            q = trans.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            curr_pose = np.array([curr_x, curr_y, yaw])

            if self.last_pose is None:
                self.add_point(curr_pose)
                return

            dist = np.linalg.norm(curr_pose[:2] - self.last_pose[:2])
            if dist > self.dist_threshold:
                self.add_point(curr_pose)

        except Exception:
            pass

    def add_point(self, pose):
        self.path_data.append(pose)
        self.last_pose = pose

    def save_current_path(self):
        if len(self.path_data) < 2:
            self.get_logger().warn("データが少なすぎるため保存しませんでした")
            return

        filename = os.path.join(self.save_dir, f"path_{self.file_count}.csv")
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'yaw'])
            writer.writerows(self.path_data)
        self.get_logger().info(f"【保存成功】 {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.is_recording:
            node.save_current_path()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
