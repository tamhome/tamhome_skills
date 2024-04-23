#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import tf, tf2_ros
import tf_conversions
import numpy as np
from time import sleep
from tamlib.utils import Logger
from sigverse_hsrlib import HSRBMoveIt, MoveJoints
from tamlib.tf import Transform, quaternion2euler, transform_pose, euler2quaternion

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, TransformStamped, Twist
from geometry_msgs.msg import Quaternion, Point
from tam_mmaction2.msg import Ax3DPoseWithLabelArray
from tam_mmaction2.msg import Ax3DPoseWithLabel, AxKeyPoint
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from .find import Find


class Deliver(Logger):
    def __init__(self):
        Logger.__init__(self, loglevel="INFO")

        self.move_joints = MoveJoints()
        self.moveit = HSRBMoveIt()
        self.tamtf = Transform()
        self.find_instance = Find()

        self.frame_map = "map"
        self.frame_odom = "odom"
        self.frame_baselink = "base_footprint"
        self.frame_rgbd = "head_rgbd_sensor_link"

        # Create a tf
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.listener = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.listener)

        self.srv_mm_run_enable = rospy.ServiceProxy("/action_recognition_server/run_enable", SetBool)

    def get_nearest_person_id(self, msg: Ax3DPoseWithLabelArray) -> int:
        """ロボットと最も近い人物のIDを取得
        Args:
            data(Ax3DPoseWithLabelArray): mmから得られたすべてのデータ
        Return:
            int: 最も近い人のID
        Sample:
            msg: Ax3DPoseWithLabelArray = rospy.wait_for_message("/mmaction2/poses/with_label", Ax3DPoseWithLabelArray)
            nearest_person_id = get_nearest_person_id(msg)
            nearest_person = msg.people[nearest_person_id]
        """
        # 鼻の座標をもとに，一番近い人を見つける
        nearest_person_id = None
        min_distance = np.inf
        for id, detect_person in enumerate(msg.people):
            if detect_person.keypoints.nose.point.z < min_distance:
                # 信頼値が低い結果ははじく
                left_shoulder = detect_person.keypoints.left_shoulder
                right_shoulder = detect_person.keypoints.right_shoulder
                shoulder_distance = self.calculate_distance(left_shoulder.point, right_shoulder.point)
                if detect_person.keypoints.left_shoulder.score < self.sholder_th or detect_person.keypoints.right_shoulder.score < self.sholder_th or shoulder_distance > 1.0:
                    continue
                min_distance = detect_person.keypoints.nose.point.z
                nearest_person_id = id

        return nearest_person_id

    def pass_to_near_person(self, timeout=100):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
            try:
                # メッセージを取得する
                msg: Ax3DPoseWithLabelArray = rospy.wait_for_message("/mmaction2/poses/with_label", Ax3DPoseWithLabelArray, timeout=3)
                self.logdebug(f"detect {len(msg.people)} person")
            except Exception as e:
                self.logwarn(e)
                self.loginfo("service call to enable mmaction")
                req = SetBoolRequest()
                req.data = True
                self.srv_mm_run_enable(req)

            # 一人も見つからなかったとき
            if len(msg.people) == 0:
                self.find_instance._base_rotation(angle=20)
                continue

            nearest_person_id = self.get_nearest_person_id(msg)
            nearest_person = msg.people[nearest_person_id]

            nose_point: Point = nearest_person.keypoints.nose.point

            # nose_pose_from_map: Pose = self.tamtf.get_pose_with_offset(
            #     target_frame=self.frame_map,
            #     source_frame=self.frame_rgbd,
            #     offset=Pose(nose_point, Quaternion(0, 0, 0, 1)),
            # )

            # nose_pose_from_baselink: Pose = self.tamtf.get_pose_with_offset(
            #     target_frame=self.frame_baselink,
            #     source_frame=self.frame_rgbd,
            #     offset=Pose(nose_point, Quaternion(0, 0, 0, 1)),
            # )

            nose_pose_from_odom: Pose = self.tamtf.get_pose_with_offset(
                target_frame=self.frame_odom,
                source_frame=self.frame_rgbd,
                offset=Pose(nose_point, Quaternion(0, 0, 0, 1)),
            )

            target_pose = Pose()
            target_pose.position.x = nose_pose_from_odom.position.x
            target_pose.position.y = nose_pose_from_odom.position.y
            target_pose.position.z = nose_pose_from_odom.position.z - 0.5

            orientation = euler2quaternion(0, -1.57, 1.57)
            target_pose.orientation.x = orientation[0]
            target_pose.orientation.y = orientation[1]
            target_pose.orientation.z = orientation[2]
            target_pose.orientation.w = orientation[3]

            self.moveit.move_to_pose(target_pose=target_pose)
            self.logsuccess("deliver: goal reached")
            return True

        self.logwarn("deliver: timeout")
        return False


if __name__ == "__main__":
    rospy.init_node('test delivery to person')
    cls = Deliver()
    cls.deliver()
