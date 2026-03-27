""" 制御フロー
ボックス持ち上げ
　　伸ばす＆吸引ポンプオン＆先端角度の調整&アームを下げる
    時間が経過したらアームを上げる＆縮める
ボックス置く
    ボックスを持った状態から伸ばす
    アームを下げる＆角度調整
    吸引ポンプオフ＆アームを初期位置＆伸縮を初期位置へ

左アームを動かす場合
右アームを動かす場合
    ボックスがおいてある段差が上か、下か
        置く動作、取る動作

左から取る


Vゴール
　　左で上段 
    右で中段
    
dynamixel_id
    4: 左アーム回転
    5: 左伸縮
    6: 左先端
    7: 右アーム回転
    8: 右伸縮
    9: 右先端


モード一覧
1 ボックス下:左:置く
2 ボックス下:左:持ち上げ
3 ボックス下:右:置く:
4 ボックス下:右:持ち上げ
5 ボックス上:左:置く
6 ボックス上:左:持ち上げ
7 ボックス上:右:置く
8 ボックス上:右:持ち上げ
9 Vゴール中段
10 Vゴール上段
"""

import rclpy
import math
import numpy as np
import time

from std_msgs.msg import UInt8, UInt16, String
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TransformStamped, Twist
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import atexit

import asyncio
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

#自作ライブラリ
from auto_robot_interfaces_v2.action import BoxArm
from dyna_interfaces.msg import DynaFeedback, DynaTarget
import os
import sys

target_dir = os.path.abspath("/home/aratahorie/ah_python_libraries")
sys.path.append(target_dir)
from ah_python_can import *

CAN_BUS = can.interface.Bus(bustype="socketcan",
                            channel="can0",
                            asynchronous=True,
                            bitrate=1000000)


# ----Config Params -----
class BoxArmController(Node):

    def __init__(self):
        super().__init__("box_arm_controller")

        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            BoxArm,
            "box_arm",
            self.execute_callback,
            callback_group=self.cb_group,
        )
        # publisherの設定
        self.dyna_pos_publisher = self.create_publisher(DynaTarget,
                                                        "/dyna_target_pos", 10)

        #吸引ポンプ立ち上げ
        set_pwm_mode(0x020, CAN_BUS)
        set_pwm_mode(0x021, CAN_BUS)

        set_goal_pwm(0x020, 0, CAN_BUS)
        set_goal_pwm(0x021, 0, CAN_BUS)

        # ----Params----
        # dyhnamixel_idリスト
        self.left_box_arm_ids = [4, 5, 6, 0x020]
        self.right_box_arm_ids = [7, 8, 9, 0x021]

        #初期化
        left_init_pos_list = [3000, 980, 1060]
        right_init_pos_list = [1900, 3600, 3000]

        #left
        self.publish_dyna_pos(self.left_box_arm_ids[0], left_init_pos_list[0])
        self.publish_dyna_pos(self.left_box_arm_ids[1], left_init_pos_list[1])
        self.publish_dyna_pos(self.left_box_arm_ids[2], left_init_pos_list[2])
        #right
        self.publish_dyna_pos(self.right_box_arm_ids[0], right_init_pos_list[0])
        self.publish_dyna_pos(self.right_box_arm_ids[1], right_init_pos_list[1])
        self.publish_dyna_pos(self.right_box_arm_ids[2], right_init_pos_list[2])

        #ボックス取得時

        #ボックスが上にあるときの位置リスト
        self.left_lift_above_pos_list = [1000, 3000, 1200, 3000, 980, 1800]
        self.left_put_above_pos_list = [0, 0, 0, 0, 0, 0]
        self.right_lift_above_pos_list = [3000, 3200, 2800, 1900, 3600, 2000]
        self.right_put_above_pos_list = [0, 0, 0, 0, 0, 0]

        # ボックスが下にあるときの位置リスト
        self.left_lift_under_pos_list = [0, 0, 0, 0, 0, 0]  #左持ち上げ
        self.left_put_under_pos_list = [0, 0, 0, 0, 0, 0]  #左置く
        self.right_lift_under_pos_list = [0, 0, 0, 0, 0, 0]  #右持ち上げ
        self.right_put_under_pos_list = [0, 0, 0, 0, 0, 0]  #右置く

        #Vゴール時
        self.vgoal_middle_pos_list = [0, 0, 0, 0, 0, 0]
        self.vgoal_above_pos_list = [0, 0, 0, 0, 0, 0]

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = target
        self.dyna_pos_publisher.publish(msg)

    # ---- Action実行メインロジック
    async def execute_callback(self, goal_handle):
        req = goal_handle.request
        res = BoxArm.Result()
        success = False

        #ボックス上
        if req.mode == 1:  #左持ち上げ
            success = await self.lift_box(self.left_box_arm_ids,
                                          self.left_lift_above_pos_list)
        elif req.mode == 2:  #左置く
            success = await self.put_box(self.left_box_arm_ids,
                                         self.left_put_above_pos_list)
        elif req.mode == 3:  #右持ち上げ
            success = await self.lift_box(self.right_box_arm_ids,
                                          self.right_lift_above_pos_list)
        elif req.mode == 4:  #右置く
            success = await self.put_box(self.right_box_arm_ids,
                                         self.right_put_above_pos_list)
        #ボックス下
        elif req.mode == 5:  #左持ち上げ
            success = await self.lift_box(self.left_box_arm_ids,
                                          self.left_lift_under_pos_list)
        elif req.mode == 6:  #左置く
            success = await self.put_box(self.left_box_arm_ids,
                                         self.left_put_under_pos_list)
        elif req.mode == 7:  #右持ち上げ
            success = await self.lift_box(self.right_box_arm_ids,
                                          self.right_lift_under_pos_list)
        elif req.mode == 8:  #右置く
            success = await self.put_box(self.right_box_arm_ids,
                                         self.right_put_under_pos_list)
        #Vゴール
        elif req.mode == 9:  #vgoal　中段 右
            success = await self.v_goal(self.right_box_arm_ids,
                                        self.vgoal_middle_pos_list)
        elif req.mode == 10:  #vgoal 上段 左
            success = await self.v_goal(self.left_box_arm_ids,
                                        self.vgoal_above_pos_list)
        res.success = success

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return res

    async def put_box(self, ids, pos_list):
        self.get_logger().info("put動作を開始します")

        #伸ばす&アームを下げる&角度調整
        self.publish_dyna_pos(ids[0], pos_list[0])
        self.publish_dyna_pos(ids[1], pos_list[1])
        self.publish_dyna_pos(ids[2], pos_list[2])
        time.sleep(4.0)

        #吸引ポンプオフ＆アームを初期位置&伸縮を初期位置
        self.publish_dyna_pos(ids[0], pos_list[3])
        self.publish_dyna_pos(ids[1], pos_list[4])
        self.publish_dyna_pos(ids[2], pos_list[5])
        set_goal_pwm(ids[3], 0, CAN_BUS)
        time.sleep(2.0)

        self.get_logger().info("動作終了、待機します")
        return True

    async def lift_box(self, ids, pos_list):
        self.get_logger().info("lift動作を開始します")

        #アームを下げる＆先端調整&伸ばす＆吸引オン
        self.publish_dyna_pos(ids[0], pos_list[0])
        self.publish_dyna_pos(ids[1], pos_list[1])
        self.publish_dyna_pos(ids[2], pos_list[2])
        set_goal_pwm(ids[3], 1000, CAN_BUS)
        time.sleep(4.0)

        #アームを上げる&縮める&角度調整
        self.publish_dyna_pos(ids[0], pos_list[3])
        self.publish_dyna_pos(ids[1], pos_list[4])
        self.publish_dyna_pos(ids[2], pos_list[5])
        time.sleep(2.0)

        self.get_logger().info("動作終了、待機します")
        return True

    async def v_goal(self, ids, pos_list):
        self.get_logger().info("v_goal動作を開始します")

        #アームを下げる＆先端調整&伸ばす
        self.publish_dyna_pos(ids[0], pos_list[0])
        self.publish_dyna_pos(ids[1], pos_list[1])
        self.publish_dyna_pos(ids[2], pos_list[2])
        time.sleep(4.0)

        #アームを上げる&縮める&角度調整
        self.publish_dyna_pos(ids[0], pos_list[3])
        self.publish_dyna_pos(ids[1], pos_list[4])
        self.publish_dyna_pos(ids[2], pos_list[5])
        set_goal_pwm(ids[3], 0, CAN_BUS)
        time.sleep(2.0)

        self.get_logger().info("動作終了、待機します")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = BoxArmController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


def stop():
    """停止モードにする"""
    set_stop_mode(0x020, CAN_BUS)
    set_stop_mode(0x021, CAN_BUS)


atexit.register(stop)

if __name__ == "__main__":
    main()
