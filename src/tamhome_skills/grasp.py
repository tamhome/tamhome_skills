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


class Grasp(Logger):
    def __init__(self):
        Logger.__init__(self, loglevel="INFO")

        self.tam_move_joints = MoveJoints()
        self.tam_moveit = HSRBMoveIt()
        self.tamtf = Transform()

        # Create a tf
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.listener = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.listener)

        self.pub_base_rot = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)

    def delete(self):
        return

    def _failure(self):
        """把持失敗
        """
        self.logwarn("grasp: failed grasping")

    def grasp_obj_by_pose(
        self,
        target_pose: Pose,
        source_frame: str = "odom",
        grasp_from="top",
        timeout=60
    ) -> bool:
        """Pose指定による把持
        Args:
            target_pose(Pose): 目標位置
            frame(str): 目標位置のベースフレーム
                odom以外では動かないので要修正
            grasp_from(str): どの方向から把持するか
                top: 上から
                fromt: 手前から
            timeout(int): タイムアウト
                defaults to 60
        Returns:
            bool: 把持に成功したかどうか（指定動作を完了したかどうか）
        """
        self.loginfo("start grasping")
        start_time = rospy.Time.now()
        target_frame = "odom"
        while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
            grasp_pose = Pose(
                target_pose.position,
                Quaternion(0, 0, 0, 1),
            )

            grasp_pose_odom: Pose = self.tamtf.get_pose_with_offset(
                target_frame=target_frame,
                source_frame=source_frame,
                offset=grasp_pose,
            )

            self.tam_move_joints.gripper(3.14)
            # 把持前の姿勢に移動
            grasp_pose_odom_pre = grasp_pose_odom
            grasp_pose_odom_pre.position.z = grasp_pose_odom.position.z + 0.1
            grasp_pose_odom_pre.orientation = euler2quaternion(0, -1.57, np.pi)
            res = self.tam_moveit.move_to_pose(grasp_pose_odom_pre, target_frame)
            rospy.sleep(1)

            # 把持姿勢に遷移
            grasp_pose_base_second = grasp_pose_odom_pre
            grasp_pose_base_second.position.z = grasp_pose_odom_pre.position.z - 0.03
            res = self.tam_moveit.move_to_pose(grasp_pose_base_second, target_frame)
            rospy.sleep(1)

            # 把持
            self.tam_move_joints.gripper(0)
            rospy.sleep(1)
            self.tam_move_joints.move_arm_by_line(+0.03, "arm_lift_joint")
            rospy.sleep(1)
            self.tam_move_joints.gripper(0)
            rospy.sleep(1)

            # 移動姿勢にする
            self.tam_move_joints.go()

            # TODO: 把持検証を実装

            self.loginfo("grasp: success")
            return True

        self.loginfo("grasp: timeout")
        return False


if __name__ == "__main__":
    rospy.init_node('test grasp')
    cls = Grasp()
    target_pose = Pose()
    target_pose.position.x = 0
    target_pose.position.y = 1
    target_pose.position.z = 0.2
    target_pose.orientation.x = 0.0
    target_pose.orientation.y = 0.0
    target_pose.orientation.z = 0.0
    target_pose.orientation.w = 1

    cls.grasp_obj_by_pose(target_pose=target_pose)
