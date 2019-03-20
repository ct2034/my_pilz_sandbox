#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, PoseArray, Quaternion
import math
import numpy as np
from pilz_robot_programming import *
import random
import rospy
import tf
import time

__REQUIRED_API_VERSION__ = "1"  # API version
VEL_SCALE = .1
ACC_SCALE = .1
RADIUS = .1
CEN_X = .2
CEN_Y = 0
CEN_Z = .8
SQ2 = 1/math.sqrt(2)

# trying circ command
def circ(r):
    start = Pose(Point(), Quaternion(0,0,0,1))
    start.position.x = CEN_X-RADIUS
    start.position.y = CEN_Y
    start.position.z = CEN_Z
    r.move(Ptp(goal=start,
               vel_scale=VEL_SCALE,
               acc_scale=ACC_SCALE))
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

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    circ(r)
