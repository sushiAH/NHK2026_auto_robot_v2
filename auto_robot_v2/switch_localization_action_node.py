import rclpy
from rclpy.node import Node
import math
import time
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from auto_robot_interfaces_v2.action import SwitchLoc
from nav2_msgs.srv import LoadMap
"""
mode 
    1: 無効化
    2: 有効化
"""


class LocalizationManager(Node):

    def __init__(self):
        super().__init__("localization_manager")
        self.cb_group = ReentrantCallbackGroup()

        # Action Server
        self._action_server = ActionServer(
            self,
            SwitchLoc,
            "switch_localization",
            self.execute_callback,
            callback_group=self.cb_group,
        )

        # 【追加】AMCL用とEKF用のパブリッシャーをそれぞれ作成
        # EKF用 (従来通り)
        self.ekf_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                  "/set_pose", 10)

        # AMCL用 (追加)
        self.amcl_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                   "/initialpose", 10)

        # Service Clients
        self.amcl_lifecycle_client = self.create_client(
            ChangeState, "/amcl/change_state", callback_group=self.cb_group)

        self.map_load_client = self.create_client(LoadMap,
                                                  "/map_server/load_map",
                                                  callback_group=self.cb_group)

    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = SwitchLoc.Result()

        self.get_logger().info(
            f"リクエスト受信: mode={req.mode}, x={req.x}, y={req.y}, yaw={req.yaw}")

        if req.mode == 1:
            # EKF単体 (AMCL停止)
            success = await self.deactivate_lidar_localization()
        elif req.mode == 2:
            # EKF + LiDAR (AMCL開始 + 初期位置設定)
            success = await self.activate_lidar_localization(
                req.x, req.y, req.yaw, req.map_path)
        else:
            success = False

        res.success = success
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    def publish_initial_pose(self, x, y, yaw):
        """AMCLとEKFの両方に初期位置を配信して同期させる"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 位置の設定
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y

        # 向きの設定 (クォータニオン) - 補正なし
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 共分散行列の設定
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.1
        msg.pose.covariance[7] = 0.1
        msg.pose.covariance[35] = 0.05

        # 【重要】両方のトピックにパブリッシュ
        #self.ekf_pose_pub.publish(msg)  # EKFのリセット用
        self.amcl_pose_pub.publish(msg)  # AMCLのリセット用

        self.get_logger().info(f"初期位置を配信しました: x={x}, y={y}, yaw={yaw}")

    async def deactivate_lidar_localization(self):
        """AMCLを無効化"""
        if not await self.change_lifecycle_state(
                Transition.TRANSITION_DEACTIVATE):
            return False
        return True

    async def activate_lidar_localization(self, x, y, yaw, map_path=None):
        """AMCLを有効化し、初期位置を設定"""
        # 地図の読み込み
        if map_path:
            await self.load_new_map(map_path)

        # AMCLのアクティブ化
        if not await self.change_lifecycle_state(Transition.TRANSITION_ACTIVATE
                                                ):
            return False

        # 指定通り time.sleep で待機
        time.sleep(1.0)

        # 初期値の配信
        self.publish_initial_pose(x, y, yaw)
        return True

    async def change_lifecycle_state(self, transition_id):
        """AMCLの状態遷移を実行"""
        if not self.amcl_lifecycle_client.wait_for_service(timeout_sec=2.0):
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id

        try:
            # 完了を待機
            future = self.amcl_lifecycle_client.call_async(req)
            response = await future
            return response.success
        except Exception as e:
            self.get_logger().error(f"Lifecycle遷移失敗: {e}")
            return False

    async def load_new_map(self, map_yaml_path):
        """map_serverに対して地図の切り替えを要求"""
        if not self.map_load_client.wait_for_service(timeout_sec=2.0):
            return False

        req = LoadMap.Request()
        req.map_url = map_yaml_path

        try:
            future = self.map_load_client.call_async(req)
            response = await future
            return True
        except Exception as e:
            self.get_logger().error(f"地図読み込み失敗: {e}")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationManager()

    # ActionとServiceを同時に扱うためMultiThreadedExecutorを使用
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
