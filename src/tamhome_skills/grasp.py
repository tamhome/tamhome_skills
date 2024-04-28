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

        self.pub_base_velocity = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)

    def delete(self):
        return

    def _failure(self):
        """把持失敗
        """
        self.logwarn("grasp: failed grasping")

    def _set_z_axis(self, pose_odom: Pose, timeout=50, tolerance=0.015, grasp_type="top") -> bool:
        """目標TFの高さとハンドの高さを揃える関数
        Args:
            pose_odom(Pose): Odom座標系のTF
        """
        if grasp_type == "top":
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
                pose_from_handpalm: Pose = self.tamtf.get_pose_with_offset(
                    target_frame="hand_palm_link",
                    source_frame="odom",
                    offset=pose_odom
                )

                distance_z = pose_from_handpalm.position.z
                self.loginfo(f"distance of z axis: {distance_z}")

                if abs(distance_z) < tolerance:
                    self.loginfo("goal reached")
                    self.tam_move_joints.move_arm_by_line(+0.0, "arm_lift_joint")
                    return True

                if distance_z < 0:
                    self.loginfo("ハンドのほうが下にあるので，上げる")
                    self.tam_move_joints.move_arm_by_line(+0.01, "arm_lift_joint")
                else:
                    self.loginfo("ハンドのほうが上にあるので，下げる")
                    self.tam_move_joints.move_arm_by_line(-0.01, "arm_lift_joint")

                rospy.sleep(0.5)

            # タイムアウトした場合
            return False
        else:
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
                pose_from_handpalm: Pose = self.tamtf.get_pose_with_offset(
                    target_frame="hand_palm_link",
                    source_frame="odom",
                    offset=pose_odom
                )

                distance_x = pose_from_handpalm.position.x
                self.loginfo(f"distance of y axis: {distance_x}")

                if abs(distance_x) < 0.015:
                    self.loginfo("goal reached")
                    self.tam_move_joints.move_arm_by_line(+0.0, "arm_lift_joint")
                    return True

                if distance_x > 0:
                    self.loginfo("ハンドのほうが下にあるので，上げる")
                    self.tam_move_joints.move_arm_by_line(+0.01, "arm_lift_joint")
                else:
                    self.loginfo("ハンドのほうが上にあるので，下げる")
                    self.tam_move_joints.move_arm_by_line(-0.01, "arm_lift_joint")

                rospy.sleep(0.5)

    def _set_xy_axis(self, pose_odom: Pose, timeout=30, grasp_type="top") -> bool:
        """目標TFのXY軸とハンドのXYを揃える関数
        Args:
            pose_odom(Pose): Odom座標系のTF
        """
        if grasp_type == "top":
            self.loginfo("reach to y axis")
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
                # y方向の誤差をなくす
                pose_from_handpalm: Pose = self.tamtf.get_pose_with_offset(
                    target_frame="hand_palm_link",
                    source_frame="odom",
                    offset=pose_odom
                )

                distance_y = abs(pose_from_handpalm.position.y)
                self.loginfo(distance_y)

                if distance_y < 0.010:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    self.pub_base_velocity.publish(twist_msg)
                    break

                twist_msg = Twist()
                if pose_from_handpalm.position.y < 0:
                    twist_msg.linear.y = +0.10
                else:
                    twist_msg.linear.y = -0.10
                self.pub_base_velocity.publish(twist_msg)
                rospy.sleep(0.1)

            self.loginfo("reach to x axis")

            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
                pose_from_handpalm: Pose = self.tamtf.get_pose_with_offset(
                    target_frame="hand_palm_link",
                    source_frame="odom",
                    offset=pose_odom
                )
                distance_x = abs(pose_from_handpalm.position.x)
                self.loginfo(distance_x)

                if distance_x < 0.010:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    self.pub_base_velocity.publish(twist_msg)
                    break

                twist_msg = Twist()
                if pose_from_handpalm.position.x < 0:
                    twist_msg.linear.x = -0.10
                else:
                    twist_msg.linear.x = +0.10

                self.pub_base_velocity.publish(twist_msg)
                rospy.sleep(0.1)

            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            self.pub_base_velocity.publish(twist_msg)
        else:
            self.loginfo("front mode")
            self.loginfo("reach to y axis")
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
                # y方向の誤差をなくす
                pose_from_handpalm: Pose = self.tamtf.get_pose_with_offset(
                    target_frame="hand_palm_link",
                    source_frame="odom",
                    offset=pose_odom
                )

                distance_y = abs(pose_from_handpalm.position.y)
                self.loginfo(distance_y)

                if distance_y < 0.010:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    self.pub_base_velocity.publish(twist_msg)
                    break

                twist_msg = Twist()
                if pose_from_handpalm.position.y < 0:
                    twist_msg.linear.y = +0.10
                else:
                    twist_msg.linear.y = -0.10
                self.pub_base_velocity.publish(twist_msg)
                rospy.sleep(0.1)

            self.loginfo("reach to x axis")

            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < rospy.Duration(timeout):
                pose_from_handpalm: Pose = self.tamtf.get_pose_with_offset(
                    target_frame="hand_palm_link",
                    source_frame="odom",
                    offset=pose_odom
                )
                pose_from_handpalm.position.z = pose_from_handpalm.position.z - 0.015
                distance_z = abs(pose_from_handpalm.position.z)
                self.loginfo(distance_z)

                if distance_z < 0.007:
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    self.pub_base_velocity.publish(twist_msg)
                    break

                twist_msg = Twist()
                if pose_from_handpalm.position.z < 0:
                    twist_msg.linear.x = -0.10
                else:
                    twist_msg.linear.x = +0.10

                self.pub_base_velocity.publish(twist_msg)
                rospy.sleep(0.1)

            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            self.pub_base_velocity.publish(twist_msg)

    def _grasp_recovery_mode(self, pose_odom: Pose, source_frame="odom") -> bool:
        """把持のデフォルト姿勢をとったうえで，base_footprintとの誤差分移動する
        Args:
            pose_odom(Pose): 対象とする物体のPose(odom座標系)
        """
        self.loginfo("enter grasp recovery mode!")

        default_grasp_joint = [0.8, -1.57, 0.0, -1.57, 0.0]
        self.tam_move_joints.move_arm_by_pose(
            default_grasp_joint[0],
            default_grasp_joint[1],
            default_grasp_joint[2],
            default_grasp_joint[3],
            default_grasp_joint[4],
        )
        rospy.sleep(2.5)

        status = self._set_z_axis(pose_odom=pose_odom)
        if status is False:
            self.loginfo("z軸高さを合わせることができませんでした")
            return False

        status = self._set_xy_axis(pose_odom=pose_odom)
        if status is False:
            self.loginfo("xy軸を合わせることができませんでした")
            return False

        rospy.sleep(1)
        self.tam_move_joints.move_arm_by_line(-0.04, "arm_lift_joint")

        return True

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

    def _move_forward(self, value=0.1) -> None:
        """台車を前に動かす
        Args:
            value(float): 移動量
        """
        start_time = rospy.Time.now()
        duration_time = value / 0.1
        while (rospy.Time.now() - start_time) < rospy.Duration(duration_time):
            twist_msg = Twist()
            twist_msg.linear.x = 0.2
            self.pub_base_velocity.publish(twist_msg)

        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        self.pub_base_velocity.publish(twist_msg)
        self.pub_base_velocity.publish(twist_msg)

    def grasp_check(self, th=0.1) -> bool:
        """ハンドの開閉角をもとにした把持チェック
        Args:
            th(float): hand_l_spring_proximal_joint の値（開閉角として利用）
        Returns:
            bool: 把持している（アームが開いている状態）ならTrue
        """
        current_joint = self.tam_move_joints.get_current_joint()
        hand_open_rad = current_joint["hand_l_spring_proximal_joint"]

        if hand_open_rad < th:
            self.logdebug("failed grasping")
            return False
        else:
            self.loginfo("success grasping")
            return True

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
            bool: 把持に成功したかどうか
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

            # self.tam_move_joints.go()
            # rospy.sleep(2)
            self.tam_move_joints.gripper(3.14)
            rospy.sleep(2)

            # TODO: 正面からの把持を実装
            if grasp_from == "front":
                pass
                # # 把持前の姿勢に移動
                # grasp_pose_odom_pre = grasp_pose_odom
                # grasp_pose_odom_pre.position.z = grasp_pose_odom.position.z + 0.1
                # grasp_pose_odom_pre.orientation = euler2quaternion(0, -1.57, np.pi)
                # res = self.tam_moveit.move_to_pose(grasp_pose_odom_pre, target_frame)
                # rospy.sleep(1)

                # # 把持姿勢に遷移
                # grasp_pose_base_second = grasp_pose_odom_pre
                # grasp_pose_base_second.position.z = grasp_pose_odom_pre.position.z - 0.03
                # res = self.tam_moveit.move_to_pose(grasp_pose_base_second, target_frame)
                # rospy.sleep(1)

            else:
                # 把持前の姿勢に移動
                grasp_pose_odom_pre = grasp_pose_odom
                grasp_pose_odom_pre.position.z = grasp_pose_odom.position.z + 0.1
                grasp_pose_odom_pre.orientation = euler2quaternion(0, -1.57, np.pi)
                # status = self.tam_moveit.move_to_pose(grasp_pose_odom_pre, target_frame)
                status = False
                rospy.sleep(1)

                if status is False:
                    recovery_status = self._grasp_recovery_mode(grasp_pose_odom_pre)
                    if recovery_status is False:
                        self.logwarn("Failed also recovery execution")
                        return recovery_status
                else:
                    # 把持姿勢に遷移
                    grasp_pose_base_second = grasp_pose_odom_pre
                    grasp_pose_base_second.position.z = grasp_pose_odom_pre.position.z - 0.03
                    status = self.tam_moveit.move_to_pose(grasp_pose_base_second, target_frame)
                    rospy.sleep(1)

            # 把持
            self.tam_move_joints.gripper(0)
            rospy.sleep(1)
            self.tam_move_joints.move_arm_by_line(+0.03, "arm_lift_joint")
            rospy.sleep(1)
            self.tam_move_joints.gripper(0)
            rospy.sleep(1)

            # 後ろに下がる
            self._move_backward(value=0.15)

            # 移動姿勢にする
            self.tam_move_joints.go()

            # 把持検証を実装
            grasp_status = self.grasp_check()
            if grasp_status is True:
                self.loginfo("grasp: success")
                return True
            else:
                self.loginfo("grasp: failure")
                return False

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
