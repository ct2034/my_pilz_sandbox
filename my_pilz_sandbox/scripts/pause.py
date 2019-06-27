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

class MoveThread(threading.Thread):
    def __init__(self, robot, cmd):
        threading.Thread.__init__(self)
        self._robot = robot
        self._cmd = cmd
        self.exception_thrown = False

    def run(self):
        rospy.logdebug("Start motion...")
        try:
            self._robot.move(self._cmd)
        except RobotMoveFailed:
            rospy.loginfo("Caught expected exception.")
            self.exception_thrown = True

# trying to pause a seq command
def pausing_a_sequence(r):
    r.move(Ptp(goal=Pose(position=Point(0.0, 0.0, .9), orientation=Quaternion(0,0,0,1)),
                    vel_scale=SLOW_VEL_SCALE,
                    acc_scale=ACC_SCALE))
    r.move(Ptp(goal=Pose(position=Point(0.0, 0.0, .9), orientation=Quaternion(0,0,0,1)),
                    vel_scale=SLOW_VEL_SCALE,
                    acc_scale=ACC_SCALE))
    print("prepared.")

    seq = Sequence()

    seq.append(Ptp(goal=Pose(position=Point(0.0, 0, .9), orientation=Quaternion(0,0,0,1)),
                    vel_scale=SLOW_VEL_SCALE,
                    acc_scale=ACC_SCALE))
    seq.append(Ptp(goal=Pose(position=Point(0.2, 0, .9), orientation=Quaternion(0,0,0,1)),
                    vel_scale=SLOW_VEL_SCALE,
                    acc_scale=ACC_SCALE),
		blend_radius=0.099)
    seq.append(Ptp(goal=Pose(position=Point(0.2, 0.2, .9), orientation=Quaternion(0,0,0,1)),
                   vel_scale=SLOW_VEL_SCALE,
                   acc_scale=ACC_SCALE),
               blend_radius=0.099)
    seq.append(Ptp(goal=Pose(position=Point(0, 0.2, .9), orientation=Quaternion(0,0,0,1)),
                   vel_scale=SLOW_VEL_SCALE,
                   acc_scale=ACC_SCALE))

    move_thread = MoveThread(r, seq)
    move_thread.start()
    for i in range(10):
        rospy.sleep(1)
        r.pause()
        rospy.sleep(.2)
        r.resume()

    move_thread.join()

# trying to pause a ptp command
def pausing_a_ptp(r):
    r.move(Ptp(goal=Pose(position=Point(-0.2, 0.0, .9), orientation=Quaternion(0,0,0,1)),
                    vel_scale=SLOW_VEL_SCALE,
                    acc_scale=ACC_SCALE))

    print("prepared.")

    ptp = Ptp(goal=Pose(position=Point(0.2, 0, .9), orientation=Quaternion(0,0,0,1)),
                    vel_scale=SLOW_VEL_SCALE,
                    acc_scale=ACC_SCALE)

    move_thread = MoveThread(r, ptp)
    move_thread.start()
    for i in range(10):
        rospy.sleep(1)
        r.pause()
        rospy.sleep(.2)
        r.resume()

    move_thread.join()

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    pausing_a_sequence(r)
    pausing_a_ptp(r)
