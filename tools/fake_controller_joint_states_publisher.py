import math
import numpy as np

import rospy

from sensor_msgs.msg import JointState

if __name__ == "__main__":
    rospy.init_node("trying")

    pub = rospy.Publisher(
        "fake_controller_joint_states",
        JointState,
        queue_size=10)

    joints = [
      "prbt_joint_1",
      "prbt_joint_2",
      "prbt_joint_3",
      "prbt_joint_4",
      "prbt_joint_5",
      "prbt_joint_6"]
    desired_v = .24
    radius = .74
    dt = .01
    n = int(2 * math.pi * radius / desired_v / dt)
    print(n)

    while not rospy.is_shutdown():
        for d in np.linspace(0, 2*math.pi, n):
            js = JointState()
            js.name = joints
            js.position = [d, math.pi/2, 0, 0, 0, 0]
            pub.publish(js)
            rospy.sleep(dt)
            if rospy.is_shutdown():
                break
