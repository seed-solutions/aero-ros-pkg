#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

class LifterStatePublisher(object):
    def __init__(self):
        self._l1 = 290.09
        self._l2 = 290.09
        self._pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self._pub_lifter = rospy.Publisher("/lifter_position", Point, queue_size=1)
        self._sub = rospy.Subscriber("/aero_joint_states", JointState, self.state_cb, queue_size=1)

    def state_cb(self, state):
        msg = JointState()
        msg.header = state.header
        msg.name = state.name
        msg.name.extend(["virtual_lifter_z_joint", "virtual_lifter_x_joint"])
        msg.position = list(state.position)
        hip = state.position[state.name.index("hip_joint")]
        knee = state.position[state.name.index("knee_joint")] 
        z = self._l1 * (math.cos(knee - hip) - 1.0) + self._l2 * (math.cos(hip) - 1.0)
        x = -self._l1 * math.sin(knee - hip) + self._l2 * math.sin(hip)
        msg.position.extend([z*0.001, x*0.001])
        self._pub.publish(msg)
        
        msg_lifter = Point();
        msg_lifter.x = x*0.001
        msg_lifter.z = (z + self._l1 + self._l2)*0.001
        self._pub_lifter.publish(msg_lifter)

if __name__ == '__main__':
    rospy.init_node('moveit_joint_state_publisher')
    LifterStatePublisher()
    rospy.spin()
