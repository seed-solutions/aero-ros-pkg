#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
import math
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from collada_model_publisher import ColladaModel
from movable_model_publisher import MovableModel

if __name__ == '__main__':
    rospy.init_node('carry_model_publisher', anonymous = True)
    modeldir = 'package://aero_samples/models/'
    rate = rospy.Rate(5)
    props = [{'filename': modeldir + 'carry.dae',
              'frame_id': 'carry_base_link',
              'parent_id': 'base_link',
              'pos': (0.6, 0.0, 0.0),
              'quat': (0, 0, math.sqrt(0.5), math.sqrt(0.5)),
              'color': (0.5, 0.5, 0.5, 1)},
             {'filename': modeldir + 'container.dae',
              'frame_id': 'container_base_link',
              'parent_id': 'carry_base_link',
              'pos': (0, 0, 0.7),
              'quat': (0, 0, math.sqrt(0.5), math.sqrt(0.5)),
              'color': (1, 0.3, 0, 1)}]

    carry = MovableModel('carry_model', props)
    sub_set_pose = rospy.Subscriber('/carry_model/set_pose',
                                    PoseStamped,
                                    carry.set_pose_callback)

    while not rospy.is_shutdown():
        carry.publish()
        rate.sleep()
