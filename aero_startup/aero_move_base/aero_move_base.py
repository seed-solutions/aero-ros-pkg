#!/usr/bin/env python
import time
import datetime
import rospy
from trajectory_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import aero_move_base_settings


def Callback(msg):
    global wheel_on
    global timestamp

    # servo wheel if not on yet
    if not wheel_on:
        power.publish(Bool(True))
        time.sleep(0.1)

    p = JointTrajectoryPoint()
    p.positions = aero_move_base_settings.msg2cmd(msg)
    p.time_from_start = rospy.Duration(0.01) # this has no meaning

    cmd = JointTrajectory()
    cmd.joint_names = aero_move_base_settings.wheel_names
    cmd.points = [p]
    pub.publish(cmd)

    #update timestamp
    timestamp = datetime.datetime.now()
    wheel_on = True


def SafeCheck(event):
    global wheel_on

    # stop wheel if no commands are received for more than 1 sec
    if (datetime.datetime.now() - timestamp).seconds >= 1 and wheel_on:
        cmd = JointTrajectory()
        cmd.joint_names = aero_move_base_settings.wheel_names

        p = JointTrajectoryPoint()
        p.positions = [0.0] * len(cmd.joint_names)
        p.time_from_start = rospy.Duration(0.01)
        cmd.points = [p]
        pub.publish(cmd)
        time.sleep(0.1)
        
        power.publish(Bool(False))
        wheel_on = False


if __name__ == '__main__':
    rospy.init_node('move_base_with_slam')

    pub = rospy.Publisher('/aero_controller/wheel_command', JointTrajectory, queue_size=10)
    power = rospy.Publisher('/aero_controller/wheel_servo', Bool, queue_size=10)
    sub = rospy.Subscriber('/y_velocity_pub', Twist, Callback)

    wheel_on = False
    timestamp = datetime.datetime.now()

    rospy.Timer(rospy.Duration(0.1), SafeCheck)
    rospy.spin()
