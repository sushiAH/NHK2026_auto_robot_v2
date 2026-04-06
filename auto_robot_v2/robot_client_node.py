"""
制御フロー:
1. 試合開始
2. ヘッドラックの前まで経路追従
3. やり取得action (コメントアウト中)
4. arucoマーカー検出待機 (コメントアウト中)
5. 検出後、段前まで移動
6. amcl無効化 (SwitchLoc)
7. 段差フロー (OverSteps, MoveOnSteps, PoseCorrection)
8. 段差降り、amcl有効化
9. 秘伝書棚前まで移動
10. Vゴール待機
"""

import os
import sys
import csv
import math
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus

# アクションインターフェースのインポート
from auto_robot_interfaces_v2.action import (OverSteps, BoxArm, PoseCorrection,
                                             MoveOnSteps, SwitchLoc, Spear,
                                             DetectAruco, IsVgoal)
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from geometry_msgs.msg import PoseStamped

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from route_searcher import *
from voice_box_lib import *


class RobotActionClient(Node):

    def __init__(self):
        super().__init__("robot_action_client")

        self.cb_group = ReentrantCallbackGroup()

        # --- Action Clientの初期化 (全て共通のグループを割り当て) ---
        self._action_client_boxarm = ActionClient(self,
                                                  BoxArm,
                                                  "box_arm",
                                                  callback_group=self.cb_group)
        self._action_client_oversteps = ActionClient(
            self, OverSteps, "over_steps", callback_group=self.cb_group)
        self._action_client_correctpos = ActionClient(
            self, PoseCorrection, "correct_pose", callback_group=self.cb_group)
        self._action_client_move_on_steps = ActionClient(
            self, MoveOnSteps, "move_on_steps", callback_group=self.cb_group)
        self._action_client_switch_localization = ActionClient(
            self,
            SwitchLoc,
            "switch_localization",
            callback_group=self.cb_group)
        self._action_client_spear = ActionClient(self,
                                                 Spear,
                                                 "spear",
                                                 callback_group=self.cb_group)
        self._action_client_detect_aruco = ActionClient(
            self, DetectAruco, "detect_aruco", callback_group=self.cb_group)
        self._action_client_is_vgoal = ActionClient(
            self, IsVgoal, "is_vgoal", callback_group=self.cb_group)
        self._action_client_path = ActionClient(self,
                                                FollowPath,
                                                "follow_path_map",
                                                callback_group=self.cb_group)

        # パブリッシャー
        self.path_pub = self.create_publisher(Path, "/visual_spline_path", 10)

        # 経路データの読み込み
        self.start_to_before_spear = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/start_to_before_spear.csv"
        )

        self.spear_to_aruco_wait = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/spear_to_aruco_wait.csv"
        )

        self.aruco_wait_to_before_steps = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/aruco_wait_to_before_steps.csv"
        )

        self.before_to_step_0 = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/before_to_step_0.csv"
        )

        self.before_to_step_1 = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/before_to_step_1.csv"
        )

        self.before_to_step_2 = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/before_to_step_2.csv"
        )

        self.step_0_to_rightside = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/step_0_to_rightside.csv"
        )

        self.step_1_to_rightside = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/step_1_to_rightside.csv"
        )

        self.step_2_to_rightside = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/step_2_to_rightside.csv"
        )

        self.step_0_to_after = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/step_0_to_after.csv"
        )

        self.step_1_to_after = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/step_1_to_after.csv"
        )

        self.step_2_to_after = self.load_path_from_csv(
            "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/path/step_2_to_after.csv"
        )

        self.map_path = "/home/aratahorie/NHK2026_auto_robot_v2/src/auto_robot_v2/map/c419_map.yaml"

        # ----Params----
        self.items = [
            [1, 0, 1],
            [0, 1, 2],
            [0, 0, 1],
            [0, 0, 0],
        ]

        self.spear_init_pos = [0.98, -0.71, -1.65]
        self.aruco_init_pos = [1.00, -0.65, 1.622]
        self.step_0_init_pos = [0, 0, 0]
        self.step_1_init_pos = [0, 0, 0]
        self.step_2_init_pos = [0, 0, 0]

        #初期化

    def load_path_from_csv(self, file_path):
        """CSVからPathメッセージを生成"""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        if not os.path.exists(file_path):
            self.get_logger().error(f"File not found: {file_path}")
            return None

        try:
            with open(file_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # ヘッダーをスキップ
                for row in reader:
                    if not row:
                        continue
                    x, y, yaw = map(float, row)
                    pose = PoseStamped()
                    pose.header = path_msg.header
                    pose.pose.position.x = x
                    pose.pose.position.y = y
                    pose.pose.orientation.z = math.sin(yaw / 2.0)
                    pose.pose.orientation.w = math.cos(yaw / 2.0)
                    path_msg.poses.append(pose)
            return path_msg
        except Exception as e:
            self.get_logger().error(f"CSV読み込みエラー: {e}")
            return None

    # --- Action実行用の共通内部メソッド ---
    async def _send_action_goal(self, client, goal_msg, action_name):
        """Actionを送信し、完了(SUCCEEDED)まで待機する共通処理"""
        self.get_logger().info(f"[{action_name}] サーバーを待機中...")
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"[{action_name}] サーバーが見つかりません")
            return False

        self.get_logger().info(f"[{action_name}] ゴール送信開始")
        send_goal_future = await client.send_goal_async(goal_msg)

        if not send_goal_future.accepted:
            self.get_logger().error(f"[{action_name}] 命令が拒否されました")
            return False

        self.get_logger().info(f"[{action_name}] 受理されました。結果を待機します...")
        result_handle = await send_goal_future.get_result_async()

        # ステータスコードのチェック
        if result_handle.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[{action_name}] 正常完了しました")
            return result_handle.result
        else:
            self.get_logger().warn(
                f"[{action_name}] 失敗しました (Status ID: {result_handle.status})")
            return None

    # --- 各アクション呼び出しメソッド群 ---

    async def send_goal(self, path_msg):
        if path_msg is None:
            return False
        self.path_pub.publish(path_msg)
        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        return await self._send_action_goal(self._action_client_path, goal_msg,
                                            "FollowPath")

    async def send_to_boxarm(self, mode):
        goal_msg = BoxArm.Goal()
        goal_msg.mode = mode
        return await self._send_action_goal(self._action_client_boxarm,
                                            goal_msg, "BoxArm")

    async def send_to_oversteps(self, mode):
        goal_msg = OverSteps.Goal()
        goal_msg.mode = mode
        return await self._send_action_goal(self._action_client_oversteps,
                                            goal_msg, "OverSteps")

    async def send_to_correctpos(self):
        goal_msg = PoseCorrection.Goal()
        return await self._send_action_goal(self._action_client_correctpos,
                                            goal_msg, "PoseCorrection")

    async def send_to_move_on_steps(self, mode):
        goal_msg = MoveOnSteps.Goal()
        goal_msg.mode = mode
        return await self._send_action_goal(self._action_client_move_on_steps,
                                            goal_msg, "MoveOnSteps")

    async def send_to_switch_loc(self,
                                 mode,
                                 x=0.0,
                                 y=0.0,
                                 yaw=0.0,
                                 map_path=""):
        goal_msg = SwitchLoc.Goal()
        goal_msg.mode = mode
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.yaw = float(yaw)
        goal_msg.map_path = str(map_path)
        return await self._send_action_goal(
            self._action_client_switch_localization, goal_msg, "SwitchLoc")

    async def send_to_spear(self):
        goal_msg = Spear.Goal()
        return await self._send_action_goal(self._action_client_spear, goal_msg,
                                            "Spear")

    async def send_to_detect_aruco(self):
        goal_msg = DetectAruco.Goal()
        return await self._send_action_goal(self._action_client_detect_aruco,
                                            goal_msg, "DetectAruco")

    async def send_to_is_vgoal(self):
        goal_msg = IsVgoal.Goal()
        return await self._send_action_goal(self._action_client_is_vgoal,
                                            goal_msg, "IsVgoal")

    async def steps_action_pattern(self, num):
        """
        ID	カテゴリ	アクション名	補足
        1	移動	段差上り	+1 の段差へ進む
        2	移動	段差降り	-1 の段差へ進む
        3	移動	そのまま移動	同じ高さのまま前進
        4	正面取得	正面取得（右腕 / 同じ）	1個目 / 正面 / 高さ±0
        5	正面取得	正面取得（右腕 / 下）	1個目 / 正面 / 高さ-1
        6	正面取得	正面取得（右腕 / 上）	1個目 / 正面 / 高さ+1
        7	正面取得	正面取得（左腕 / 同じ）	2個目 / 正面 / 高さ±0
        8	正面取得	正面取得（左腕 / 下）	2個目 / 正面 / 高さ-1
        9	正面取得	正面取得（左腕 / 上）	2個目 / 正面 / 高さ+1
        10	左側取得	左側取得（右腕 / 同じ）	1個目 / 左側 / 高さ±0
        11	左側取得	左側取得（右腕 / 下）	1個目 / 左側 / 高さ-1
        12	左側取得	左側取得（右腕 / 上）	1個目 / 左側 / 高さ+1
        13	左側取得	左側取得（左腕 / 同じ）	2個目 / 左側 / 高さ±0
        14	左側取得	左側取得（左腕 / 下）	2個目 / 左側 / 高さ-1
        15	左側取得	左側取得（左腕 / 上）	2個目 / 左側 / 高さ+1
        16	右側取得	右側取得（右腕 / 同じ）	1個目 / 右側 / 高さ±0
        17	右側取得	右側取得（右腕 / 下）	1個目 / 右側 / 高さ-1
        18	右側取得	右側取得（右腕 / 上）	1個目 / 右側 / 高さ+1
        19	右側取得	右側取得（左腕 / 同じ）	2個目 / 右側 / 高さ±0
        20	右側取得	右側取得（左腕 / 下）	2個目 / 右側 / 高さ-1
        21	右側取得	右側取得（左腕 / 上）	2個目 / 右側 / 高さ+1
        22	廃棄	廃棄（左腕）	1回目の廃棄
        23	廃棄	廃棄（右腕）	2回目の廃棄
        24	準備	位置確認・真ん中移動	移動完了直後の姿勢制御
        25	進入	0列目進入（取得あり）	進入時に正面の箱を取得
        26	進入	1列目進入（取得あり）	進入時に正面の箱を取得
        27	進入	2列目進入（取得あり）	進入時に正面の箱を取得
        28	進入	0列目進入（取得なし）	取得せずに進入
        29	進入	1列目進入（取得なし）	取得せずに進入
        30	進入	2列目進入（取得なし）	取得せずに進入
        31	最終移動	0列目から地点移動	退出後、指定地点へ移動
        32	最終移動	1列目から地点移動	退出後、指定地点へ移動
        33	最終移動	2列目から地点移動	退出後、指定地点へ移動
        """

        if num == 1:
            #段差上り
            await self.send_to_move_on_steps(8)  #中央から段差のぼり位置まで移動
            await self.send_to_oversteps(1)  #段差上り
        elif num == 2:
            #段差降り
            await self.send_to_move_on_steps(8)
            await self.send_to_oversteps(2)  #段差降り
        elif num == 3:
            #段差そのまま
            await self.send_to_move_on_steps(8)
            await self.send_to_oversteps(3)  #段差そのまま
        elif num == 4:
            #正面取得 ,右腕、同じ
            await self.send_to_move_on_steps(4)
            await self.send_to_boxarm(12)
            await self.send_to_move_on_steps(7)

        elif num == 5:
            #正面取得 ,右腕、下
            await self.send_to_move_on_steps(4)
            await self.send_to_boxarm(4)
            await self.send_to_move_on_steps(7)

        elif num == 6:
            #正面取得 ,右腕、上
            await self.send_to_move_on_steps(4)
            await self.send_to_boxarm(8)
            await self.send_to_move_on_steps(7)

        elif num == 7:
            #正面取得 ,左腕、同じ
            await self.send_to_move_on_steps(4)
            await self.send_to_boxarm(10)
            await self.send_to_move_on_steps(7)

        elif num == 8:
            #正面取得 ,左腕、下
            await self.send_to_move_on_steps(4)
            await self.send_to_boxarm(2)
            await self.send_to_move_on_steps(7)

        elif num == 9:
            #正面取得 ,左腕、上
            await self.send_to_move_on_steps(4)
            await self.send_to_boxarm(6)
            await self.send_to_move_on_steps(7)

        elif num == 10:
            #左側取得 ,右腕、同じ
            await self.send_to_move_on_steps(2)
            await self.send_to_boxarm(12)
            await self.send_to_move_on_steps(5)

        elif num == 11:
            #左側取得 ,右腕、下
            await self.send_to_move_on_steps(2)
            await self.send_to_boxarm(4)
            await self.send_to_move_on_steps(5)

        elif num == 12:
            #左側取得 ,右腕、上
            await self.send_to_move_on_steps(2)
            await self.send_to_boxarm(8)
            await self.send_to_move_on_steps(5)

        elif num == 13:
            #左側取得 ,左腕、同じ
            await self.send_to_move_on_steps(2)
            await self.send_to_boxarm(10)
            await self.send_to_move_on_steps(5)

        elif num == 14:
            #左側取得 ,左腕、下
            await self.send_to_move_on_steps(2)
            await self.send_to_boxarm(2)
            await self.send_to_move_on_steps(5)

        elif num == 15:
            #左側取得 ,左腕、上
            await self.send_to_move_on_steps(2)
            await self.send_to_boxarm(6)
            await self.send_to_move_on_steps(5)

        elif num == 16:
            #右側取得 ,右腕、同じ
            await self.send_to_move_on_steps(3)
            await self.send_to_boxarm(12)
            await self.send_to_move_on_steps(6)

        elif num == 17:
            #右側取得 ,右腕、下
            await self.send_to_move_on_steps(3)
            await self.send_to_boxarm(4)
            await self.send_to_move_on_steps(6)

        elif num == 18:
            #右側取得 ,右腕、上
            await self.send_to_move_on_steps(3)
            await self.send_to_boxarm(8)
            await self.send_to_move_on_steps(6)

        elif num == 19:
            #右側取得 ,左腕、同じ
            await self.send_to_move_on_steps(3)
            await self.send_to_boxarm(10)
            await self.send_to_move_on_steps(6)

        elif num == 20:
            #右側取得 ,左腕、下
            await self.send_to_move_on_steps(3)
            await self.send_to_boxarm(2)
            await self.send_to_move_on_steps(6)

        elif num == 21:
            #右側取得 ,左腕、上
            await self.send_to_move_on_steps(3)
            await self.send_to_boxarm(6)
            await self.send_to_move_on_steps(6)

        elif num == 22:
            #廃棄、左
            await self.send_to_boxarm(14)
        elif num == 23:
            #廃棄、右
            await self.send_to_boxarm(13)
        elif num == 24:
            #位置確認、真ん中移動
            await self.send_to_correctpos()
            await self.send_to_move_on_steps(1)
        elif num == 25:
            #0列目侵入、取得あり
            #高さ1
            await self.send_goal(self.before_to_step_0)
            await self.send_to_boxarm(8)
            await self.send_goal(self.step_0_to_rightside)
            await self.send_to_switch_loc(1)
            await self.send_to_oversteps(1)
        elif num == 26:
            #1列目侵入、取得あり
            #高さ0
            await self.send_goal(self.before_to_step_1)
            await self.send_to_boxarm(12)
            await self.send_goal(self.step_1_to_rightside)
            await self.send_to_switch_loc(1)
            await self.send_to_oversteps(3)
        elif num == 27:
            #2列目侵入、取得あり
            #高さ1
            await self.send_goal(self.before_to_step_2)
            await self.send_to_boxarm(8)
            await self.send_goal(self.step_2_to_rightside)
            await self.send_to_switch_loc(1)
            await self.send_to_oversteps(1)
        elif num == 28:
            #0列目侵入、取得なし
            await self.send_goal(self.before_to_step_0)
            await self.send_goal(self.step_0_to_rightside)
            await self.send_to_switch_loc(1)
            await self.send_to_oversteps(1)
        elif num == 29:
            #1列目侵入、取得なし
            await self.send_goal(self.before_to_step_1)
            await self.send_goal(self.step_1_to_rightside)
            await self.send_to_switch_loc(1)
            await self.send_to_oversteps(3)
        elif num == 30:
            #2列目侵入、取得なし
            await self.send_goal(self.before_to_step_2)
            await self.send_goal(self.step_2_to_rightside)
            await self.send_to_switch_loc(1)
            await self.send_to_oversteps(1)
        elif num == 31:
            #退出地点移動、0列目
            await self.send_to_switch_loc(2, self.step_0_init_pos[0],
                                          self.step_0_init_pos[1],
                                          self.step_0_init_pos[2],
                                          self.map_path)
            await self.send_goal()
        elif num == 32:
            #退出地点移動、1列目
            await self.send_to_switch_loc(2, self.step_1_init_pos[0],
                                          self.step_1_init_pos[1],
                                          self.step_1_init_pos[2],
                                          self.map_path)
            await self.send_goal()
        elif num == 33:
            #退出地点移動、2列目
            await self.send_to_switch_loc(2, self.step_2_init_pos[0],
                                          self.step_2_init_pos[1],
                                          self.step_2_init_pos[2],
                                          self.map_path)
            await self.send_goal()

    # --- メインシーケンス ---
    async def run_robot_sequence(self):
        self.get_logger().info("--- ロボットシーケンス開始 ---")
        #speak("ロボットを起動します")
        #await self.send_to_correctpos()

        #やり取得シークエンス
        await self.send_goal(self.start_to_before_spear)  #スタートからやり
        await self.send_to_switch_loc(1)  #amcl無効化
        await self.send_to_spear()  #やり取得
        await self.send_to_switch_loc(
            2,
            self.spear_init_pos[0],  #amcl有効化
            self.spear_init_pos[1],
            self.spear_init_pos[2],
            self.map_path)

        await self.send_goal(self.spear_to_aruco_wait)
        await self.send_to_switch_loc(1)  #amcl無効化
        await self.send_to_detect_aruco()  #aruco検出待ち
        await self.send_to_switch_loc(
            2,
            self.aruco_init_pos[0],  #amcl有効化
            self.aruco_init_pos[1],
            self.aruco_init_pos[2],
            self.map_path)
        await self.send_goal(self.aruco_wait_to_before_steps)

        #段差超え(テスト)
        #await self.send_goal(self.spear_to_before_steps) #槍から段差前まで
        await self.steps_action_pattern(28)  #スタート、段差超え
        await self.steps_action_pattern(24)  #位置確認、真ん中移動

        #段差超えシークエンス(本番)
        #await self.send_goal()  #グリッドの前に移動
        #route = calc_on_step_sequence(self.items)
        #for i in range(0, len(route)):
        #    await self.steps_action_pattern(route[i])

        #Vゴールシークエンス
        #await self.send_goal()  #棚の前まで移動
        #await self.send_to_boxarm(15)  #中段を入れる
        #await self.send_to_is_vgoal()  #高さ待ち
        #await self.send_to_boxarm(16)  #中段を入れる

        self.get_logger().info("--- 全シーケンス正常終了 ---")


def main(args=None):
    rclpy.init(args=args)

    node = RobotActionClient()

    # MultiThreadedExecutor: 複数のスレッドでコールバック（Action結果受信など）を処理可能にする
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    # シーケンスをバックグラウンドタスクとして作成
    # これにより spin() を実行しながらシーケンスを並行して進める
    sequence_task = executor.create_task(node.run_robot_sequence)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("キーボード割り込みにより停止します")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
