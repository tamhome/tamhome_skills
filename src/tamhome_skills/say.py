#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from tamlib.utils import Logger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class HSRBSay(Logger):
    def __init__(self):
        Logger.__init__(self)

    def say(txt: str, type=1) -> bool:
        """"""
        pass


if __name__ == "__main__":
    cls = HSRBSay()
    cls.say("test message")
