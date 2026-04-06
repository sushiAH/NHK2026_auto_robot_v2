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
2個目は右から取る


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


mode 
    1: ボックス下:左:置く
    2: ボックス下:左:持ち上げ
    3: ボックス下:右:置く:
    4: ボックス下:右:持ち上げ

    5: ボックス上:左:置く
    6: ボックス上:左:持ち上げ
    7: ボックス上:右:置く
    8: ボックス上:右:持ち上げ

    9: ボックス同じ:左:置く
    10: ボックス同じ:左:持ち上げ
    11: ボックス同じ:右:置く
    12: ボックス同じ:右:持ち上げ

    13: ボックス破棄:右
    14: ボックス破棄:左

    15: Vゴール中段
    16: Vゴール上段

left
  4: 880 3430 (水平、垂直)
  5: 1000 4000 (縮む、伸びる)
  6: 1040 2040 (水平、垂直)

right
  7: 3430 880(水平、垂直) 
  8: 3300 200 (縮む、伸びる)
  9: 3100 2100 (水平、垂直)
    
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


def degree_to_dyna_pos(degree, origin_pos_list):
    resolution = (origin_pos_list[1] - origin_pos_list[0]) / 90.0
    dyna_pos = (degree * resolution) + origin_pos_list[0]
    return dyna_pos


def extend_ratio_to_dyna_pos(ratio, origin_pos_list):
    resolution = origin_pos_list[1] - origin_pos_list[0]
    dyna_pos = (ratio * resolution) + origin_pos_list[0]
    return dyna_pos


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
        #dynamixel 原点リスト
        self.origin_pos_dict = {
            4: [880, 3430],  # [水平、垂直]
            5: [1000, 4000],  # [縮む、伸びる]
            6: [1040, 2040],  # [水平、垂直]
            7: [3430, 880],
            8: [3400, 100],
            9: [3100, 2100],
        }

        #left
        self.set_rot(self.left_box_arm_ids[0], 90)
        self.set_extend(self.left_box_arm_ids[1], 0.1)
        self.set_hand(self.left_box_arm_ids[2], 0)

        #right
        self.set_rot(self.right_box_arm_ids[0], 60)
        self.set_extend(self.right_box_arm_ids[1], 0.1)
        self.set_hand(self.right_box_arm_ids[2], 0)

        #ボックス取得時
        #ボックスが上にあるときの位置リスト
        self.left_lift_above_pos_list = [20, 0.8, -20, 90, 0.2, 90]
        self.left_put_above_pos_list = [20, 0.8, -20, 90, 0.2, 90]
        self.right_lift_above_pos_list = [20, 0.8, -20, 60, 0.2, 90]
        self.right_put_above_pos_list = [20, 0.8, -20, 60, 0.2, 90]
        # ボックスが下にあるときの位置リスト
        self.left_lift_under_pos_list = [-20, 0.8, 20, 90, 0.2, 90]  #左持ち上げ
        self.left_put_under_pos_list = [-20, 0.8, 20, 90, 0.2, 90]  #左置く
        self.right_lift_under_pos_list = [-20, 0.8, 20, 60, 0.2, 90]  #右持ち上げ
        self.right_put_under_pos_list = [-20, 0.8, 20, 60, 0.2, 90]  #右置く
        # ボックスが同じ高さにあるときの位置リスト
        self.left_lift_same_pos_list = [10, 0.8, -10, 90, 0.2, 90]  #左持ち上げ
        self.left_put_same_pos_list = [10, 0.8, -10, 90, 0.2, 90]  #左置く
        self.right_lift_same_pos_list = [10, 0.8, -10, 60, 0.2, 90]  #右持ち上げ
        self.right_put_same_pos_list = [10, 0.8, -10, 60, 0.2, 90]  #右置く
        #ボックス破棄の位置リスト
        self.left_destruct_pos_list = [100, 0.8, 90, 90, 0.2, 90]
        self.right_destruct_pos_list = [100, 0.8, 90, 60, 0.2, 90]
        #Vゴール時
        self.vgoal_middle_pos_list = [30, 0.8, 10, 90, 0.2, 90]
        self.vgoal_above_pos_list = [60, 0.8, 10, 60, 0.2, 90]

    def publish_dyna_pos(self, id, target):
        msg = DynaTarget()
        msg.id = id
        msg.target = int(target)
        self.dyna_pos_publisher.publish(msg)

    def set_rot(self, id, target_degree):
        target_dyna_pos = degree_to_dyna_pos(target_degree,
                                             self.origin_pos_dict[id])
        self.publish_dyna_pos(id, target_dyna_pos)

    def set_extend(self, id, target_ratio):
        target_dyna_pos = extend_ratio_to_dyna_pos(target_ratio,
                                                   self.origin_pos_dict[id])
        self.publish_dyna_pos(id, target_dyna_pos)

    def set_hand(self, id, target_degree):
        target_dyna_pos = degree_to_dyna_pos(target_degree,
                                             self.origin_pos_dict[id])
        self.publish_dyna_pos(id, target_dyna_pos)

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
        #ボックスそのまま
        elif req.mode == 8:
            success = await self.lift_box(self.left_box_arm_ids,
                                          self.left_lift_same_pos_list)
        elif req.mode == 9:
            success = await self.put_box(self.left_box_arm_ids,
                                         self.left_put_same_pos_list)
        elif req.mode == 10:
            success = await self.lift_box(self.right_box_arm_ids,
                                          self.right_lift_same_pos_list)
        elif req.mode == 11:
            success = await self.put_box(self.right_box_arm_ids,
                                         self.right_put_same_pos_list)
        #ボックス破棄
        elif req.mode == 12:
            success = await self.destruct_box(self.left_box_arm_ids,
                                              self.left_destruction_pos_list)
        elif req.mode == 13:
            success = await self.destruct_box(self.right_box_arm_ids,
                                              self.right_destruction_pos_list)
        #Vゴール
        elif req.mode == 14:  #vgoal　中段 右
            success = await self.v_goal(self.right_box_arm_ids,
                                        self.vgoal_middle_pos_list)
        elif req.mode == 15:  #vgoal 上段 左
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
        self.set_extend(ids[1], pos_list[1])
        self.set_hand(ids[2], pos_list[2])
        time.sleep(1.0)
        self.set_rot(ids[0], pos_list[0])
        time.sleep(4.0)

        #吸引ポンプオフ＆アームを初期位置&伸縮を初期位置
        set_goal_pwm(ids[3], 0, CAN_BUS)
        self.set_rot(ids[0], pos_list[3])
        self.set_hand(ids[2], pos_list[5])
        time.sleep(4.0)
        self.set_extend(ids[1], pos_list[4])

        self.get_logger().info("動作終了、待機します")
        return True

    async def lift_box(self, ids, pos_list):
        self.get_logger().info("lift動作を開始します")

        #アームを下げる＆先端調整&伸ばす＆吸引オン
        self.set_extend(ids[1], pos_list[1])
        self.set_hand(ids[2], pos_list[2])
        time.sleep(1.0)
        self.set_rot(ids[0], pos_list[0])
        set_goal_pwm(ids[3], 1000, CAN_BUS)
        time.sleep(5.0)

        #アームを上げる&縮める&角度調整
        self.set_rot(ids[0], pos_list[3])
        time.sleep(2.0)
        self.set_hand(ids[2], pos_list[5])
        time.sleep(2.0)
        self.set_extend(ids[1], pos_list[4])

        self.get_logger().info("動作終了、待機します")
        return True

    async def destruct_box(self, ids, pos_list):
        self.get_logger().info("破棄動作を開始します")

        self.set_rot(ids[0], pos_list[0])
        self.set_extend(ids[1], pos_list[1])
        self.set_hand(ids[2], pos_list[2])
        time.sleep(5.0)

        set_goal_pwm(ids[3], 0, CAN_BUS)
        self.publish_dyna_pos(ids[0], pos_list[3])
        self.publish_dyna_pos(ids[1], pos_list[4])
        self.publish_dyna_pos(ids[2], pos_list[5])
        time.sleep(5.0)

        self.get_logger().info("動作終了、待機します")
        return True

    async def v_goal(self, ids, pos_list):
        self.get_logger().info("v_goal動作を開始します")

        #アームを下げる＆先端調整&伸ばす
        self.set_rot(ids[0], pos_list[0])
        self.set_extend(ids[1], pos_list[1])
        self.set_hand(ids[2], pos_list[2])
        time.sleep(5.0)

        #アームを上げる&縮める&角度調整
        self.set_rot(ids[0], pos_list[3])
        self.set_extend(ids[1], pos_list[4])
        self.set_hand(ids[2], pos_list[5])
        set_goal_pwm(ids[3], 0, CAN_BUS)
        time.sleep(5.0)

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
