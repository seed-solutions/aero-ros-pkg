#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

class LifterStatePublisher(object):
    def __init__(self):
        self._l1 = 250.0
        self._l2 = 250.0
        self._pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self._sub = rospy.Subscriber("/aero_joint_states", JointState, self.state_cb, queue_size=1)

    def state_cb(self, state):
        msg = JointState()
        msg.header = state.header
        msg.name = state.name
        hip_id = state.name.index("hip_joint")
        knee_id = state.name.index("knee_joint")
        if "ankle_joint" in state.name :
            ankle_id = state.name.index("ankle_joint")
            msg.position = list(state.position)
        hip = state.position[state.name.index("hip_joint")]
        knee = state.position[state.name.index("knee_joint")] 
        z = self._l1 * (math.cos(knee - hip) - 1.0) + self._l2 * (math.cos(hip) - 1.0)
        x = -self._l1 * math.sin(knee - hip) + self._l2 * math.sin(hip)
        msg.name[hip_id] = "virtual_lifter_x_joint"
        msg.name[knee_id] = "virtual_lifter_z_joint"
        del msg.name[ankle_id]
        msg.position[hip_id] = x*0.001
        msg.position[knee_id] = z*0.001
        del msg.position[ankle_id]
        self._pub.publish(msg)
        
if __name__ == '__main__':
    rospy.init_node('moveit_joint_state_publisher')
    LifterStatePublisher()
    rospy.spin()
