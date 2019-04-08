#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, PoseArray, Quaternion
import math
import numpy as np
from pilz_robot_programming import *
import random
import rospy
import time

__REQUIRED_API_VERSION__ = "1"  # API version
VEL_SCALE = .3
ACC_SCALE = .1
RADIUS = .1
CEN_X = .2
CEN_Y = 0
CEN_Z = .8
SQ2 = 1/math.sqrt(2)
RND = .2

# trying circ command
def circ(r):
    for _ in range(10):
        pre_start = Pose(Point(), Quaternion(0,0,0,1))
        pre_start.position.x = CEN_X-RADIUS + RND*random.random()-RND/2
        pre_start.position.y = CEN_Y + RND*random.random()-RND/2
        pre_start.position.z = CEN_Z + RND*random.random()-RND/2
        r.move(Ptp(goal=pre_start,
                   vel_scale=VEL_SCALE,
                   acc_scale=ACC_SCALE))
        print("pre Start: (%.3f, %.3f, %.3f)" %(
            pre_start.position.x,
            pre_start.position.y,
            pre_start.position.z)
        )
        start = Pose(Point(), Quaternion(0,0,0,1))
        start.position.x = CEN_X-RADIUS
        start.position.y = CEN_Y
        start.position.z = CEN_Z
        r.move(Ptp(goal=start,
                   vel_scale=VEL_SCALE,
                   acc_scale=ACC_SCALE))
        print("Start: (%.3f, %.3f, %.3f)" %(
            start.position.x,
            start.position.y,
            start.position.z)
        )
        interim = Pose(Point(), Quaternion(0,0,0,1))
        interim.position.x = CEN_X+RADIUS
        interim.position.y = CEN_Y
        interim.position.z = CEN_Z
        interim_p = interim.position
        goal = Pose(Point(), Quaternion(0,0,0,1))
        goal.position.x = CEN_X+SQ2*RADIUS
        goal.position.y = CEN_Y-SQ2*RADIUS
        goal.position.z = CEN_Z
        r.move(Circ(goal=goal,
                    interim=interim_p,
                    vel_scale=VEL_SCALE,
                    acc_scale=ACC_SCALE))
        print("Goal: (%.3f, %.3f, %.3f)" %(
            goal.position.x,
            goal.position.y,
            goal.position.z)
            )
        pose = r.get_current_pose()
        print("Pose: (%.3f, %.3f, %.3f)" %(
            pose.position.x,
            pose.position.y,
            pose.position.z)
        )
        assert abs(goal.position.y - pose.position.y) < 1E-3, "Wrong Point"

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    circ(r)
