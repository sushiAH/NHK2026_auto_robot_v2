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
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from auto_robot_interfaces_v2.action import OverSteps, BoxArm, PoseCorrection, MoveOnSteps, SwitchLoc, Spear, DetectAruco
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
import asyncio


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
        self._action_client_path = ActionClient(self, FollowPath, "follow_path")

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

    async def send_to_spear(self, mode):
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

    async def send_to_switch_loc(self, mode):
        self._action_client_switch_localization.wait_for_server()

        goal_msg = SwitchLoc.Goal()
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

    async def send_to_detect_aruco(self, mode):
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

    async def send_goal(self, path_msg):
        self._action_client_path.wait_for_server()

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller.id = "FollowPath"
        await self._action_client.send_goal_async(goal_msg)

    # ロボット制御シーケンスはここに記述する
    async def run_robot_sequence(self):

        #await self.send_to_spear()
        #await self.send_to_detect_aruco()
        #await self.send_to_correctpos()
        #await self.send_to_move_on_steps(2)
        #await self.send_to_boxarm(2)
        #await send_goal(self,path_msg)
        await self.send_to_oversteps(2)
        self.get_logger().info("シーケンス終了")


#命令を実行
def main():
    rclpy.init()
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
