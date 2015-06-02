#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped

class AeroHeadTFBroadcaster:
    def __init__(self):
        self.position = (0, 0, 0)
        self.orientation = (0, 0, 0, 1)
        self.tfb = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber("head_pos", PoseStamped,
                                    self.pose_callback)

    def pose_callback(self, msg):
        self.position = (msg.pose.position.x,
                         msg.pose.position.y,
                         msg.pose.position.z)
        self.orientation = (msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w)
    def broadcast(self):
        self.tfb.sendTransform(self.position,
                               self.orientation,
                               rospy.Time.now(),
                               "head_link",
                               "base_link")

if __name__ == "__main__":
    rospy.init_node("head_tf")
    head_tf = AeroHeadTFBroadcaster()
    while not rospy.is_shutdown():
        head_tf.broadcast()
        rospy.sleep(0.1)

