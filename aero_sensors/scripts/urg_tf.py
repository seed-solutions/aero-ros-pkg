#!/usr/bin/env python
import rospy
import tf
from dynamixel_msgs.msg import JointState

class URGTFPublisher():
    def __init__(self):
        rospy.init_node('urg_tf_publisher', anonymous=True)
        self.br = tf.TransformBroadcaster()
        self.sub = rospy.Subscriber("/tilt_controller/state",
                               JointState,
                               self.joint_state_callback)

    def joint_state_callback(self, msg):
        tm = msg.header.stamp
        angle = msg.current_pos
        self.br.sendTransform(
            (0.09, 0, 0.028),
            tf.transformations.quaternion_from_euler(angle, 0, 0),
            rospy.Time.now(),
            "urg_frame",
            "base_link")

if __name__=='__main__':
    urg_tf = URGTFPublisher()
    rospy.spin()
