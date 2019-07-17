#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, PoseArray, Quaternion
import math
import numpy as np
from pilz_robot_programming import *
import random
import rospy
import time

__REQUIRED_API_VERSION__ = "1"  # API version

n = 100;
d = .1;

def rand_pm_d():
    return 2 * d * (random.random() - .5)

# making a grid
def grid(r):
    sp = PoseArray()
    fp = PoseArray()
    sp.header.frame_id = "prbt_base_link"
    fp.header.frame_id = "prbt_base_link"
    pub_fp = rospy.Publisher("failed_poses", PoseArray)
    pub_sp = rospy.Publisher("succesful_poses", PoseArray)
    for x in np.linspace(-.5, .5, 10):
        for y in np.linspace(-.5, .5, 10):
            for z in np.linspace(0, 1, 10):
                # pose = r.get_current_pose()
                # joint_states = r.get_current_joint_states()
                # new_pose = Pose(Point(
                #     pose.position.x + rand_pm_d(),
                #     pose.position.y + rand_pm_d(),
                #     pose.position.z + rand_pm_d(),
                #     ), pose.orientation)
                new_pose = Pose(Point(x, y, z), Quaternion())
                print(new_pose)
                try:
                    r.move(Ptp(goal=new_pose,
                               vel_scale=0.2))
                    sp.poses.append(new_pose)
                    pub_sp.publish(sp)
                except Exception as e:
                    fp.poses.append(new_pose)
                    pub_fp.publish(fp)
                    print(e)

if __name__ == "__main__":
    # init a rosnode
    rospy.init_node('robot_program_node')

    # initialisation
    r = Robot(__REQUIRED_API_VERSION__)  # instance of the robot

    # start the main program
    grid(r)
