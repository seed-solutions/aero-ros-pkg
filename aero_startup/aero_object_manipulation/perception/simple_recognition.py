#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import roslib
import sys
import tf
import jsk_recognition_msgs.msg

def handle_simple_recognition(msg):
    print "in handle_simple_recognition"
    pos_x = msg.boxes[0].pose.position.x
    pos_y = msg.boxes[0].pose.position.y
    pos_z = msg.boxes[0].pose.position.z
    print "sample position: (%f, %f, %f)" % (pos_x, pos_y, pos_z)
    br = tf.TransformBroadcaster()
    br.sendTransform((pos_x, pos_y, pos_z),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "sample",
                     "ps4eye_frame")

if __name__ == '__main__':
    rospy.init_node('hsi_cloud_segmentation')
    print "hsi_cloud_segmentation"
    rospy.Subscriber('/stereo/hsi_color_filter/boxes',
                     jsk_recognition_msgs.msg.BoundingBoxArray,
                     handle_simple_recognition
                     )
    rospy.spin()
