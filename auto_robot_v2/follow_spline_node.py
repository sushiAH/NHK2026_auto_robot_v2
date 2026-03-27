"""
1点目、2点目はclicked_pointを受け取る
3点目のみgoal_poseをうけとり、最終的にyaw角も合わせる
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation as R
import math


class RvizSplineFollower(Node):

    def __init__(self):
        super().__init__("rviz_spline_follower")

        self.points = []

        # publish pointの受取
        self.point_sub = self.create_subscription(PointStamped,
                                                  "/clicked_point",
                                                  self.point_callback, 10)

        # 3点目を受け取るサブスクライバ
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose",
                                                 self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, "/visual_spline_path", 10)
        self._action_client = ActionClient(self, FollowPath, "follow_path_map")

        self.get_logger().info("RVizでpublish_pointを3回クリックしてください")

    def point_callback(self, msg):
        """rviz_publish_pointがクリックされると呼び出し

        Args:
            msg
        """

        if len(self.points) < 2:
            self.points.append([msg.point.x, msg.point.y])
            self.get_logger().info(f"地点追加: {len(self.points)}点目")

    def goal_callback(self, msg):
        if len(self.points) != 2:
            self.get_logger().warn("先に通過点を2点クリックしてください")
            return
        self.points.append([msg.pose.position.x, msg.pose.position.y])

        q = msg.pose.orientation
        final_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        self.get_logger().info("終点と向きを受理、経路を生成します")
        path_msg = self.generate_path(self.points, final_yaw)
        self.path_pub.publish(path_msg)
        self.send_goal(path_msg)
        self.points = []

    def generate_path(self, pts, final_yaw):
        """pathをスプライン補完で生成

        Args:
            pts ([]): [list]
        """
        pts = np.array(pts)
        t = np.array([0, 1, 2])
        cs = CubicSpline(t, pts, bc_type="natural")

        t_fine = np.linspace(0, 2, 50)
        smooth_pts = cs(t_fine)

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(smooth_pts)):
            pose = PoseStamped()
            pose.header = path.header
            x, y = float(smooth_pts[i][0]), float(smooth_pts[i][1])
            pose.pose.position.x = x
            pose.pose.position.y = y

            if i < len(smooth_pts) - 1:
                next_x = float(smooth_pts[i + 1][0])
                next_y = float(smooth_pts[i + 1][1])
                #yaw = math.atan2(next_y - y, next_x - x)  #全方向移動台車であれば、yawは指定しない
            else:
                yaw = final_yaw
                quat = R.from_euler("z", yaw).as_quat()

                (
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ) = quat

            path.poses.append(pose)

        return path

    def send_goal(self, path_msg):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            return
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = "FollowPath"
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RvizSplineFollower())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
