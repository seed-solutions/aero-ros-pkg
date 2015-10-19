#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import roslib
import sys
import tf
import jsk_recognition_msgs.msg

def handle_tomato_pose(msg):
    print "in handle_tomato_pose"
    pos_x = msg.boxes[0].pose.position.x
    pos_y = msg.boxes[0].pose.position.y
    pos_z = msg.boxes[0].pose.position.z
    print "Tomato position: (%f, %f, %f)" % (pos_x, pos_y, pos_z)
    br = tf.TransformBroadcaster()
    br.sendTransform((pos_x, pos_y, pos_z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "tomato",
                     "ps4eye_frame")

if __name__ == '__main__':
    rospy.init_node('test_tomato')
    print "test_tomato"
    rospy.Subscriber('/stereo/hsi_color_filter/boxes',
                     jsk_recognition_msgs.msg.BoundingBoxArray,
                     handle_tomato_pose
                     )
    rospy.spin()
