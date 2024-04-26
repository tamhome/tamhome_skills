#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import tf, tf2_ros
import numpy as np
from time import sleep
from tamlib.utils import Logger
from sigverse_hsrlib import HSRBMoveIt, MoveJoints
from tamlib.tf import Transform, quaternion2euler, transform_pose, euler2quaternion

from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose2D

from tam_object_detection.srv import LangSamObjectDetectionService, LangSamObjectDetectionServiceRequest, LangSamObjectDetectionServiceResponse


class Put(Logger):
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
        self.pub_base_velocity = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
        self.srv_detection = rospy.ServiceProxy("/hsr_head_rgbd/lang_sam/object_detection/service", LangSamObjectDetectionService)

    def _move_backward(self, value=0.1) -> None:
        """台車を後ろに下げる
        Args:
            value(float): 移動量
        """
        start_time = rospy.Time.now()
        duration_time = value / 0.1
        while (rospy.Time.now() - start_time) < rospy.Duration(duration_time):
            twist_msg = Twist()
            twist_msg.linear.x = -0.2
            self.pub_base_velocity.publish(twist_msg)

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        self.pub_base_velocity.publish(twist_msg)
        self.pub_base_velocity.publish(twist_msg)

    def put_for_furniture(self, furniture_name: str, max_distance=2.0, timeout=30):
        """家具を認識し，その上に置く
        Args:
            furniture_name(str): 認識対象，配置対象とする家具の名前
            max_distance(float): 対象家具との距離
        Returns:
            bool: 成功したかどうか
        """
        confidence_th = 0.9
        target_frame = "odom"
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
            det_req = LangSamObjectDetectionServiceRequest(
                confidence_th=confidence_th,
                iou_th=confidence_th,
                max_distance=max_distance,
                use_latest_image=True,
                prompt=furniture_name
            )
            detections = self.srv_detection(det_req).detections
            self.logtrace(detections)

            # 認識に失敗した場合
            if detections.is_detected is False:
                self.loginfo(f"Could not detect target object: {furniture_name}")
                confidence_th = confidence_th - 0.1
                continue
            else:
                # 認識に成功した場合
                pose = detections.pose[0]
                pose_odom = self.tamtf.get_pose_with_offset(
                    target_frame="odom",
                    source_frame="head_rgbd_sensor_link",
                    offset=pose,
                )

                if pose_odom is not None:
                    res = self.put_by_pose(pose=pose_odom)
                    return res

                else:
                    self.loginfo(f"cannot get tf by target_fram: {target_frame}")
                    continue

        # タイムアウト
        return False

    def put_by_pose(self, pose: Pose, target_frame="odom") -> bool:
        """姿勢を指定して配置する
        """
        # rosparamに設定できる形式に変換
        pose_by_list = [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w)
        ]
        rospy.set_param("/tamhome_skills/put_object/furniture_pose", pose_by_list)

        # # 逆運動学をときやすい姿勢に変更
        # self.tam_move_joints.go()
        # rospy.sleep(2)

        # 配置の前姿勢に移動
        put_pose_odom = pose
        put_pose_odom.position.z = pose.position.z + 0.1
        put_pose_odom.orientation = euler2quaternion(0, -1.57, np.pi)
        status = self.tam_moveit.move_to_pose(put_pose_odom, target_frame)

        if status is False:
            return False
        else:
            rospy.sleep(1)

            self.tam_move_joints.gripper(3.14)
            rospy.sleep(1)

            self._move_backward(value=0.1)

            return True
