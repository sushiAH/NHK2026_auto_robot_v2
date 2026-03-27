"""制御フロー
試合開始
ヘッドラックの前まで経路追従
やり取得action
arucoマーカ-検出待機
検出後、段前まで移動
amcl無効化
段差フロ-
段差降り、amcl有効化、
秘伝書棚前まで移動
Vゴール待機
"""
import os
import csv
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from auto_robot_interfaces_v2.action import OverSteps, BoxArm, PoseCorrection, MoveOnSteps, SwitchLoc, Spear, DetectAruco, IsVgoal
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped
import asyncio
import math


class RobotActionClient(Node):

    def __init__(self):
        super().__init__("client")
        self._action_client_boxarm = ActionClient(self, BoxArm, "box_arm")
        self._action_client_oversteps = ActionClient(self, OverSteps,
                                                     "over_steps")
        self._action_client_correctpos = ActionClient(self, PoseCorrection,
                                                      "correct_pose")
        self._action_client_move_on_steps = ActionClient(
            self, MoveOnSteps, "move_on_steps")
        self._action_client_switch_localization = ActionClient(
            self, SwitchLoc, "switch_loc")
        self._action_client_spear = ActionClient(self, Spear, "spear")
        self._action_client_detect_aruco = ActionClient(self, DetectAruco,
                                                        "detect_aruco")
        self._action_client_is_vgoal = ActionClient(self, IsVgoal, "is_vgoal")
        self._action_client_path = ActionClient(self, FollowPath,
                                                "follow_path_map")

        self.path_pub = self.create_publisher(Path, "/visual_spline_path", 10)
        #---- Params ----
        self.start_to_steps = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/start_to_steps.csv"
        )

    def load_path_from_csv(self, file_path):
        """
        CSVファイルを読み込んで nav_msgs/msg/Path を作成する
        CSV形式: x, y, yaw (1行目はヘッダーを想定)
        """
        path_msg = Path()
        # タイムスタンプは送信直前に設定するのが一般的
        path_msg.header.frame_id = "map"
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

    async def send_to_boxarm(self, mode):
        self._action_client_boxarm.wait_for_server()
        goal_msg = BoxArm.Goal()
        goal_msg.mode = mode
        send_goal_future = await self._action_client_boxarm.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return
        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

        if (result_future.result.success):
            self.get_logger().info("成功")
        else:
            self.get_logger().info("失敗")

    async def send_to_oversteps(self, mode):
        self._action_client_oversteps.wait_for_server()
        goal_msg = OverSteps.Goal()
        goal_msg.mode = mode
        send_goal_future = await self._action_client_oversteps.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return
        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

        if (result_future.result.success):
            self.get_logger().info("成功")
        else:
            self.get_logger().info("失敗")

    async def send_to_correctpos(self):
        self._action_client_correctpos.wait_for_server()

        goal_msg = PoseCorrection.Goal()
        send_goal_future = await self._action_client_correctpos.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return
        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

        if (result_future.result.success):
            self.get_logger().info("成功")
        else:
            self.get_logger().info("失敗")

    async def send_to_move_on_steps(self, mode):
        self._action_client_move_on_steps.wait_for_server()

        goal_msg = MoveOnSteps.Goal()
        goal_msg.mode = mode
        send_goal_future = await self._action_client_move_on_steps.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return
        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

        if (result_future.result.success):
            self.get_logger().info("成功")
        else:
            self.get_logger().info("失敗")

    async def send_to_spear(self):
        self._action_client_spear.wait_for_server()

        goal_msg = Spear.Goal()
        send_goal_future = await self._action_client_spear.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return
        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

        if (result_future.result.success):
            self.get_logger().info("成功")
        else:
            self.get_logger().info("失敗")

    async def send_to_switch_loc(
        self,
        mode,
        x=None,
        y=None,
        yaw=None,
        map_path=None,
    ):
        self._action_client_switch_localization.wait_for_server()

        goal_msg = SwitchLoc.Goal()
        goal_msg.mode = mode
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.yaw = yaw
        goal_msg.map_path = map_path

        send_goal_future = await self._action_client_switch_localization.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return
        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

        if (result_future.result.success):
            self.get_logger().info("成功")
        else:
            self.get_logger().info("失敗")

    async def send_to_detect_aruco(self):
        self._action_client_detect_aruco.wait_for_server()

        goal_msg = DetectAruco.Goal()
        send_goal_future = await self._action_client_detect_aruco.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return
        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

        if (result_future.result.success):
            self.get_logger().info("成功")
        else:
            self.get_logger().info("失敗")

    async def send_to_is_vgoal(self):
        self._action_client_is_vgoal.wait_for_server()

        goal_msg = IsVgoal.Goal()
        send_goal_future = await self._action_client_is_vgoal.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return

        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

        if (result_future.result.success):
            self.get_logger().info("成功")
        else:
            self.get_logger().info("失敗")

    async def send_goal(self, path_msg):
        self._action_client_path.wait_for_server()

        self.path_pub.publish(path_msg)
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        send_goal_future = await self._action_client_path.send_goal_async(
            goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().info("命令拒否")
            return
        self.get_logger().info("命令受理")

        result_future = await send_goal_future.get_result_async()

    # ロボット制御シーケンスはここに記述する
    async def run_robot_sequence(self):

        await self.send_goal(self.start_to_steps)
        await self.send_to_oversteps(1)

        #やり取得
        #await self.send_goal(start_to_spear_path)
        #await self.send_to_switch_loc(1)
        #await self.send_to_spear()
        #await self.send_to_detect_aruco()
        #await self.send_to_switch_loc(2,0,0,0,map_path)

        #段差フロー
        #await self.send_goal(spear_to_steps_path)
        #await self.send_to_switch_loc(1)
        #await self.send_to_oversteps(1)
        #await self.send_to_correctpos()
        #await self.send_to_move_on_steps(2) #左へ
        #await self.send_to_boxarm(2) #左で下を持ち上げ
        #await self.send_to_oversteps(2) #段差降り
        #await self.send_to_switch_loc(2,0,0,0,map_path)

        #Vゴールフロー
        #await self.send_goal(steps_to_arena_path)
        #await self.send_to_boxarm(9) #Vゴール中段シュート 右
        #await self.send_to_is_vgoal()
        #await self.send_to_boxarm(10) #Vゴール上段シュート 左

        self.get_logger().info("シーケンス終了")


#命令を実行
def main(args=None):
    rclpy.init(args=args)
    node = RobotActionClient()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    executor.create_task(node.run_robot_sequence())

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
"""------ROS2の仕組みについて-------
spin()はコールバックの実行を管理する。メッセージが届いたらコールバックを実行するイベント処理
multithread executorを使用することで、コールバックを複数スレッドで実行可能。これにより、sleep()関数などを使用しても、ブロッキングされない。
非同期処理は、単一スレッドで行うから、仮に、内部でsleep()などをするとブロッキングされてしまう。
非同期処理=ノンブロッキングではない

"""
