#!/usr/bin/env python

import rospy
import laser_geometry

from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan

class URGPCLPublisher():
    def __init__(self):
        rospy.init_node('urg_pcl_publisher', anonymous=True)
        self.pub = rospy.Publisher("/urg_points", PointCloud2, queue_size = 1)
        self.sub = rospy.Subscriber("/scan",
                               LaserScan,
                               self.laser_callback)
        self.laser_geom = laser_geometry.LaserProjection()

    def laser_callback(self, msg):
        points = self.laser_geom.projectLaser(msg)
        self.pub.publish(points)

if __name__=='__main__':
    urgpcl = URGPCLPublisher()
    rospy.spin()
