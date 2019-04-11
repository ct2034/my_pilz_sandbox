#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, PoseArray, Quaternion
import math
import numpy as np
from pilz_robot_programming import *
import random
import rospy
import time

__REQUIRED_API_VERSION__ = "1"  # API version
SLOW_VEL_SCALE = .1
ACC_SCALE = .1
GRIPPER_POSE_CLOSED = 0.001
GRIPPER_POSE_OPEN = 0.029

# trying circ command
def sequence(r):
    seq = Sequence()
    seq.append(Gripper(goal=GRIPPER_POSE_CLOSED))

    seq.append(Lin(goal=Pose(position=Point(0.2, 0, 1)),
                    vel_scale=SLOW_VEL_SCALE,
                    acc_scale=ACC_SCALE))
    seq.append(Lin(goal=Pose(position=Point(0.2, 0.2, 1)),
                   vel_scale=SLOW_VEL_SCALE,
                   acc_scale=ACC_SCALE),
               blend_radius=0.19)
    seq.append(Lin(goal=Pose(position=Point(0, 0.2, 1)),
                   vel_scale=SLOW_VEL_SCALE,
                   acc_scale=ACC_SCALE))

    seq.append(Gripper(goal=GRIPPER_POSE_OPEN))
    seq.append(Gripper(goal=GRIPPER_POSE_CLOSED))

    print("start sequence 1")
    r.move(seq)
    print("end sequence 1")

    seq = Sequence()
    seq.append(Gripper(goal=GRIPPER_POSE_CLOSED))

    seq.append(Lin(goal=Pose(position=Point(0, 0.2, 1)),
        vel_scale=SLOW_VEL_SCALE,
        acc_scale=ACC_SCALE))
    seq.append(Lin(goal=Pose(position=Point(0.2, 0.2, 1)),
                   vel_scale=SLOW_VEL_SCALE,
                   acc_scale=ACC_SCALE),
               blend_radius=0.19)
    seq.append(Gripper(goal=GRIPPER_POSE_OPEN))
    seq.append(Lin(goal=Pose(position=Point(0.2, 0, 1)),
                   vel_scale=SLOW_VEL_SCALE,
                   acc_scale=ACC_SCALE))

    seq.append(Gripper(goal=GRIPPER_POSE_CLOSED))

    print("start sequence 2")
    r.move(seq)
    print("end sequence 2")

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    sequence(r)
