#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from collada_model_publisher import ColladaModel

## @brief This class defines geometrical object model using MarkerArray with pose
class MovableModel(ColladaModel):
    ## @brief constructor
    ## @param name name of publisher, it is also used as name space
    ## @param props properties for each models,
    ## are described as list of dict, [
    ##  'filename': file name of collada (inc. .dae),
    ##  'frame_id': frame id,
    ##  'parent_id': parent id,
    ##  'pos' : position as tuple (x, y, z)
    ##  'quat' : quaternion as tuple (x, y, z, w)
    ##  'color': RGBA color as tuple (R, G, B, A)
    ## ]
    def __init__(self, name, props):
        super(MovableModel, self).__init__(name, props)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.props = props

    ## @brief set pose
    ## @param i index in props
    ## @param pos position (x, y, z)
    ## @param quat quaternion (x, y, z, w)
    def set_pose(self, i, pos, quat):
        self.props[i]['pos'] = pos
        qx, qy, qz, qw = quat
        nq = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if nq > 0.0:
            self.props[i]['quat'] = (qx / nq, qy / nq, qz / nq, qw / nq)

    ## @brief change parent_id and set pose in new parent_id
    ## @param i index in props
    ## @param parent_id parent frame
    ## @param pos position (x, y, z) in new parent
    ## @param quat quaternion (x, y, z, w) in new parent
    def set_parent(self, i, parent_id, pos, quat):
        self.props[i]['parent_id'] = parent_id
        self.set_pose(i, pos, quat)

    ## @brief set pose for callback,
    ##  this callback is NOT REGISTERED AUTOMATICALLY
    ##  because the class allows multiple instance generation.
    ##  To use this, please add manually in main function.
    ## @param msg geometry_msgs/PoseStamped,
    ##  if msg.header.frame_id = parent_id, call set_pose,
    ##  else call set_parent and set frame_id as new parent_id
    def set_pose_callback(self, msg):
        pos = (msg.pose.position.x,
               msg.pose.position.y,
               msg.pose.position.z)
        quat = (msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w)
        if self.props[0]['parent_id'] == msg.header.frame_id:
            self.set_pose(0, pos, quat)
        else:
            self.set_parent(0, msg.header.frame_id, pos, quat)

    ## @brief publish markers with tf (if pos, quat, and parent_id are exist)
    def publish(self):
        super(MovableModel, self).publish()
        for prop in self.props:
            if 'parent_id' in prop and 'pos' in prop and 'quat' in prop:
                self.tf_broadcaster.sendTransform(
                    prop['pos'], prop['quat'],
                    rospy.Time.now(),
                    prop['frame_id'],
                    prop['parent_id'])
