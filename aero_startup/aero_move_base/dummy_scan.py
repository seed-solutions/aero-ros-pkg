#!/usr/bin/env python
# -*- coding: utf-8 -*-
## @author Hiroaki Yaguchi JSK

import rospy
from sensor_msgs.msg import LaserScan

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


if __name__ == '__main__':
    rospy.init_node('dummy_scan', anonymous = True)
    pub = rospy.Publisher('scan', LaserScan, queue_size = 1)
    scan = LaserScan()
    scan.header.frame_id = 'wheels_base_laser_link'
    scan.angle_min = -1.57079637051
    scan.angle_max = 1.57079637051
    scan.angle_increment = 0.00436332309619
    scan.time_increment = 1.73611151695e-05
    scan.scan_time = 0.0250000003725
    scan.range_min = 0.019999999553
    scan.range_max = 30.0
    # ranges_size = 721
    ranges_size = int(1.0 + (scan.angle_max - scan.angle_min)
                      / scan.angle_increment)
    for i in range(ranges_size):
        scan.ranges.append(10.0)

    rate = rospy.Rate(40)
    while not rospy.is_shutdown():
        scan.header.stamp = rospy.get_rostime()
        pub.publish(scan)
        rate.sleep()
