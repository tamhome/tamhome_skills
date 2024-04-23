#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import tf
import sys
import math
import rospy
import numpy as np
# import moveit_msgs.msg
import tf2_ros
from typing import Tuple, List
from tamlib.utils import Logger
from tamlib.tf import Transform

from sigverse_hsrlib import HSRBMoveIt, MoveJoints

from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import DisplayTrajectory

from tam_object_detection.srv import LangSamObjectDetectionService, LangSamObjectDetectionServiceRequest, LangSamObjectDetectionServiceResponse
# from tam_object_detection.srv import ObjectDetectionService, ObjectDetectionServiceRequest


class Find(Logger):

    def __init__(self):
        Logger.__init__(self, loglevel="INFO")

        # TF2リスナーのセットアップ
        self.tf_listener = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_listener)
        self.broadcaster = tf2_ros.TransformBroadcaster()

        self.tam_move_joints = MoveJoints()
        self.tam_moveit = HSRBMoveIt()
        self.tamtf = Transform()

        # ros interface
        self.pub_base_rot = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
        self.srv_detection = rospy.ServiceProxy("/hsr_head_rgbd/lang_sam/object_detection/service", LangSamObjectDetectionService)

    def delete(self):
        return

    def _base_rotation(self, angle=90) -> None:
        """台車をその場で回転させる関数
        Args:
            angle(float): 回転させる角度
        """
        start_time = rospy.Time.now()
        duration_time = angle / 45
        while (rospy.Time.now() - start_time) < rospy.Duration(duration_time):
            twist_msg = Twist()
            twist_msg.angular.z = 0.4
            self.pub_base_rot.publish(twist_msg)

        twist_msg = Twist()
        twist_msg.angular.z = 0.0
        self.pub_base_rot.publish(twist_msg)
        self.pub_base_rot.publish(twist_msg)

    def _head_rotation(self, points=[90, 0, -90, -180]):
        """指定されたポイントの順番にしたがって首を回転させる
        Args:
            point(List): 指定ジョイント角度のリスト
                defaults to [-180, -90, 0, 90]
                value can set -200 ~ 90
        Returs:
            bool: 移動に成功したかどうか
        """
        try:
            for point in points:
                self.loginfo(f"target head_pan_joint: {point}")
                current_joint = self.tam_move_joints.get_current_joint()
                current_head_tile_joint = current_joint["head_tilt_joint"]
                self.tam_move_joints.move_head(np.deg2rad(point), current_head_tile_joint)
                distance = current_joint["head_pan_joint"] - np.deg2rad(point)
                rospy.sleep(abs(distance)*2.5)
        except Exception as e:
            self.logwarn(e)
            self.logwarn("invalid joint in head_tilte_joint")

    def do_default_recog_pose(self, pan=-0.4) -> None:
        """デフォルトの認識姿勢に遷移
        Args:
            pan(float): 首の角度
                defaults to -0.4[rad]
        Returns:
            None
        """
        self.tam_move_joints.go()
        rospy.sleep(2)
        self.tam_move_joints.move_head(0, np.deg2rad(-40))
        rospy.sleep(2)
        return

    def calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        """
        Calculate the Euclidean distance between two Pose objects.

        Args:
            pose1 (Pose): The first pose.
            pose2 (Pose): The second pose.

        Returns:
            float: The Euclidean distance between pose1 and pose2.
        """
        x1, y1, z1 = pose1.position.x, pose1.position.y, pose1.position.z
        x2, y2, z2 = pose2.position.x, pose2.position.y, pose2.position.z

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def broadcast_tf(self, position: Tuple[float, float, float], orientation: Tuple[float, float, float, float], target_frame: str, base_frame="odom", loop_num=30) -> None:
        """TFの配信をする関数
        Args:
            position (List[float, float, float]): 目標フレームの位置を表す3要素のタプル (x, y, z)
            orientation (List[float, float, float, float]): 目標フレームの姿勢を表すクォータニオン (x, y, z, w)
            target_frame (str): 変換情報が適用される子フレームの名前
            base_frame (str, optional): 変換情報の基準となる親フレームの名前。デフォルトは 'odom'
            loop_num(int): 30

        Example:
            broadcast_tf([1.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0], "target_frame", "odom")
        """
        transform = TransformStamped()

        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = base_frame
        transform.child_frame_id = target_frame
        transform.transform.translation.x = position[0]
        transform.transform.translation.y = position[1]
        transform.transform.translation.z = position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]

        # 発行するトランスフォームを送信
        for _ in range(loop_num):
            self.broadcaster.sendTransform(transform)

    def find_obj(
        self,
        target_object: str,
        max_distance=0,
        confidence_th=0.8,
        show_tf=True,
        tf_name=None,
        base_frame="odom",
        is_rotation=False,
        return_all=False
    ) -> Pose:
        """対象物体を見つけ，その位置を返す関数
        Args:
            target_object(str): 対象の物体
            max_distance(float): デプスのしきい値（0でしきい値処理なし）
                defaults is 0
            confidence_th(float): 認識のしきい値
                defaults is 0.5
            show_tf(bool): 対象物体のTFを表示
            tf_name(str): 表示するTFの名前
                defaults is target_object's name
            base_frame(str): 親フレーム
            rotation(bool): その場で90度ずつ回転し，対象物体がないか探すモード
                defaults is False
            return_all(bool): サービスレスポンスをそのまま返すかどうかを選択可能
                defaults is False
        Return:
            Pose: 対象物体の位置
        """
        is_detected = False
        self.loginfo(f"find: target object name is {target_object}")
        base_rotation_angle = 0
        confidence_th += 0.1

        while not is_detected:
            # 首を降っても見つからなかった場合
            confidence_th = confidence_th - 0.1
            if base_rotation_angle != 0:
                self._base_rotation(base_rotation_angle)

            # どのように首を振るか決定
            if is_rotation:
                rotation_points = [0, 30, -30]
            else:
                rotation_points = [999]

            # 首を振りながら検出を実行
            for point in rotation_points:
                if point != 999:
                    self._head_rotation(points=[point])

                det_req = LangSamObjectDetectionServiceRequest(
                    confidence_th=confidence_th,
                    iou_th=0.8,
                    max_distance=max_distance,
                    use_latest_image=True,
                    prompt=target_object
                )
                detections = self.srv_detection(det_req).detections
                self.logtrace(detections)

                # 認識に失敗した場合
                if detections.is_detected is False:
                    self.loginfo(f"Could not detect target object: {target_object}")
                    base_rotation_angle = 5
                    continue
                else:
                    break

            # 認識に失敗した場合は台車を移動させて再度認識をかける
            if detections.is_detected is False:
                continue

            # 認識に成功した場合
            if show_tf:
                pass

            pose = detections.pose[0]
            pose_odom = self.tamtf.get_pose_with_offset(
                target_frame="odom",
                source_frame="head_rgbd_sensor_link",
                offset=pose,
            )
            if pose_odom is not None:
                # rosparamに設定できる形式に変換
                pose_by_list = [
                    float(pose_odom.position.x),
                    float(pose_odom.position.y),
                    float(pose_odom.position.z),
                    float(pose_odom.orientation.x),
                    float(pose_odom.orientation.y),
                    float(pose_odom.orientation.z),
                    float(pose_odom.orientation.w)
                ]
                rospy.set_param("/tamhome_skills/object_detection/current_pose_odom", pose_by_list)

            if return_all:
                return detections
            else:
                self.logsuccess("find: find target object!")
                return detections.pose[0]


if __name__ == '__main__':
    try:
        rospy.init_node("moveit_test")
        cls = HSRBMoveIt()
        target_frame = "target_frame"
        cls.move_to_tf_target(target_frame)

    except rospy.ROSInterruptException:
        pass
