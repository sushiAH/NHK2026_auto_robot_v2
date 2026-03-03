import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from auto_robot_interfaces_v2.action import OverSteps, BoxArm, PoseCorrection, MoveOnSteps
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

    async def run_robot_sequence(self):

        await self.send_to_correctpos()
        await self.send_to_move_on_steps(2)
        #await self.send_to_boxarm(2)

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
