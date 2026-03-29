import rclpy
from rclpy.node import Node
import math
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from auto_robot_interfaces_v2.action import SwitchLoc
from nav2_msgs.srv import LoadMap
import asyncio
import time
"""
mode 
    1: 無効化
    2: 有効化
"""


class LocalizationManager(Node):

    def __init__(self):
        super().__init__("localization_manager")
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            SwitchLoc,
            "switch_localization",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      "/set_pose", 10)

        #serviceの作成
        self.amcl_lifecycle_client = self.create_client(ChangeState,
                                                        "/amcl/change_state")

        self.map_load_client = self.create_client(LoadMap,
                                                  "/map_server/load_map")

    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = SwitchLoc.Result()
        success = False
        message = ""

        #ekf単体
        if req.mode == 1:
            success = await self.deactivate_lidar_localization()

        #ekf + lidar
        elif req.mode == 2:
            success = await self.activate_lidar_localization(
                req.x, req.y, req.yaw, req.map_path)

        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    def publish_initial_pose(self, x, y, yaw):
        #初期位置配信
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.05

        self.initial_pose_pub.publish(msg)

    #amcl無効化
    async def deactivate_lidar_localization(self):
        if not await self.change_lifecycle_state(
                Transition.TRANSITION_DEACTIVATE):
            return False
        return True

    #amcl有効化
    async def activate_lidar_localization(self, x, y, yaw, map_path=None):
        #地図の読み込み
        await self.load_new_map(map_path)
        if not await self.change_lifecycle_state(Transition.TRANSITION_ACTIVATE
                                                ):
            return False
        time.sleep(1.0)

        #初期値のpublish
        self.publish_initial_pose(x, y, yaw)
        return True

    async def change_lifecycle_state(self, transition_id):
        if not self.amcl_lifecycle_client.wait_for_service(timeout_sec=2.0):
            return False
        req = ChangeState.Request()
        req.transition.id = transition_id
        try:
            await self.amcl_lifecycle_client.call_async(req)
            return True
        except Exception as e:
            self.get_logger().error(f"Lifecycle transition failed: {e}")
            return False

    async def load_new_map(self, map_yaml_path):
        """map_serverに対して地図の切り替えを要求する"""
        if not self.map_load_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("map_server load_map service not available")
            return False

        req = LoadMap.Request()
        req.map_url = map_yaml_path  # YAMLファイルのフルパスを指定

        try:
            await self.map_load_client.call_async(req)
            self.get_logger().info(f"Successfully loaded map: {map_yaml_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
"""ROS2 lifecycle nodeについて
serviceのserverとして実装されている
/node名/change_state
に対して、active deactiveなどの状態を送ればよい
立ち上げはLifecycleManagerで立ち上げる"""
