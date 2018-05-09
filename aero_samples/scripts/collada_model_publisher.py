#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
from visualization_msgs.msg import Marker, MarkerArray

## @brief This class defines geometrical object model using MarkerArray
class ColladaModel(object):
    ## @brief constructor
    ## @param name name of publisher, it is also used as name space
    ## @param props properties for each markers,
    ## are described as list of dict, [
    ##  'filename': file name of collada (inc. .dae),
    ##  'frame_id': frame id,
    ##  'color': RGBA color as tuple (R, G, B, A)
    ## ]
    def __init__(self, name, props):
        self.name = name
        self.markers = MarkerArray()
        i = 0
        for prop in props:
            marker = self.init_marker(
                prop['filename'], prop['frame_id'], i)
            if 'color' in prop:
                marker.color.r = prop['color'][0]
                marker.color.g = prop['color'][1]
                marker.color.b = prop['color'][2]
                marker.color.a = prop['color'][3]
            self.markers.markers.append(marker)
            i += 1
        self.pub = rospy.Publisher('shop_models', MarkerArray, queue_size = 1)

    ## @brief initialize visualization_msgs/Marker
    ## @param dae_name collada file name (.dae)
    ## @param frame_id frame id
    ## @param marker_id marker id (Int)
    ## @return visualization_msgs/Marker
    def init_marker(self, dae_name, frame_id, marker_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = dae_name
        marker.mesh_use_embedded_materials = True
        marker.id = marker_id
        marker.action = 0
        marker.ns = self.name
        marker.pose.orientation.w = 1
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        return marker

    def publish(self):
        for marker in self.markers.markers:
            marker.header.stamp = rospy.get_rostime()
        self.pub.publish(self.markers)

## Usage:
if __name__ == '__main__':
    rospy.init_node('collada_model_publisher', anonymous = True)
    rate = rospy.Rate(1)
    dae = ColladaModel(
        'shelf_model',
        [{'filename': 'package://aero_maps/models/shelf.dae',
          'frame_id': 'shelf_base_link',
          'color': (0.9, 0.9, 0.9, 1)
      }])
    while not rospy.is_shutdown():
        dae.publish()
        rate.sleep()
