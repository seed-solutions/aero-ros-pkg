#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
import tf
import math
import numpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData

# [INFO] [WallTime: 1508148230.485155] msg:
# [INFO] [WallTime: 1508148230.485310]   angle_min: -1.57079637051
# [INFO] [WallTime: 1508148230.485466]   angle_max: 1.57079637051
# [INFO] [WallTime: 1508148230.485623]   angle_increment: 0.00436332309619
# [INFO] [WallTime: 1508148230.485770]   time_increment: 1.73611151695e-05
# [INFO] [WallTime: 1508148230.485927]   scan_time: 0.0250000003725
# [INFO] [WallTime: 1508148230.486063]   size(ranges): 721
# [INFO] [WallTime: 1508148230.486205]   approx. size: 721
#range_min: 0.019999999553
#range_max: 30.0

class DummyScan:
    def __init__(self):
        self.pub = rospy.Publisher('scan', LaserScan, queue_size = 1)
        self.tf_listener = tf.TransformListener()

        self.laser_height = 0.2344

        self.scan = LaserScan()
        self.scan.header.frame_id = 'wheels_base_laser_link'
        self.scan.angle_min = -1.57079637051
        self.scan.angle_max = 1.57079637051
        self.scan.angle_increment = 0.00436332309619
        self.scan.time_increment = 1.73611151695e-05
        self.scan.scan_time = 0.0250000003725
        self.scan.range_min = 0.019999999553
        self.scan.range_max = 30.0
        # ranges_size = 721
        self.ranges_size = int(1.0 +
                               (self.scan.angle_max - self.scan.angle_min)
                               / self.scan.angle_increment)

        # subscribe map once
        self.sub_map = rospy.Subscriber('map', OccupancyGrid,
                                        self.map_to_points)

    def make_constant_range(self):
        for i in range(self.ranges_size):
            self.scan.ranges.append(10.0)

    ## @brief map to points
    ## @param msg nav_msgs/OccupancyGrid
    def map_to_points(self, msg):
        map_info = msg.info
        cr = math.cos(math.acos(map_info.origin.orientation.w) * 2.0)
        sr = math.sqrt(1.0 - cr * cr)
        self.mapgrid_list = []
        for y in range(map_info.height):
            yidx = y * map_info.width
            my = y * map_info.resolution + map_info.origin.position.y
            for x in range(map_info.width):
                val = msg.data[x + yidx]
                if val >= 90:
                    mx = x * map_info.resolution + map_info.origin.position.x
                    pt = numpy.array([mx * cr - my * sr,
                                      mx * sr + my * cr,
                                      self.laser_height])
                    self.mapgrid_list.append(pt)
        self.sub_map.unregister()

    def make_map_range(self):
        self.tf_listener.waitForTransform(
            '/map',
            'wheels_base_laser_link',
            rospy.Time(),
            rospy.Duration(10.0))
        laser_pos, laser_quat = self.tf_listener.lookupTransform(
            '/map',
            'wheels_base_laser_link',
            rospy.Time())
        laser_theta = math.acos(laser_quat[3]) * 2.0

        self.scan.ranges = []
        for i in range(self.ranges_size):
            self.scan.ranges.append(0.0)

        for mp in self.mapgrid_list:
            lx = mp[0] - laser_pos[0]
            ly = mp[1] - laser_pos[1]
            lrange = math.sqrt(lx * lx + ly * ly)
            ltheta = math.atan2(ly, lx) - laser_theta
            if ltheta > math.pi:
                ltheta = ltheta - (math.pi * 2)
            elif ltheta < -math.pi:
                ltheta = ltheta + (math.pi * 2)

            if ltheta >= self.scan.angle_min and ltheta <= self.scan.angle_max:
                li = (ltheta - self.scan.angle_min) / self.scan.angle_increment
                if self.scan.ranges[li] < 1e-5:
                    self.scan.ranges[li] = lrange
                elif self.scan.ranges[li] > lrange:
                    self.scan.ranges[li] = lrange


    def publish(self):
        self.scan.header.stamp = rospy.get_rostime()
        self.pub.publish(self.scan)

if __name__ == '__main__':
    rospy.init_node('dummy_scan', anonymous = True)
    dummy_scan = DummyScan()
    # dummy_scan.make_constant_range()

    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        dummy_scan.make_map_range()
        dummy_scan.publish()
        rate.sleep()
