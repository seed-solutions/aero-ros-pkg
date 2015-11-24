#!/usr/bin/env python

from math import *
import time

import roslib
import rospy

import move_base_msgs
import move_base_msgs.msg
import geometry_msgs
import geometry_msgs.msg
import actionlib
import tf

client = actionlib.SimpleActionClient('move_base/goal',
                                      move_base_msgs.msg.MoveBaseAction)
listener = None

def init():
    rospy.init_node('move_base_py')
    client.wait_for_server()
    global listener
    listener = tf.TransformListener()

def go_pos(_x, _y, _theta):
    p = geometry_msgs.msg.Point(x=_x, y=_y, z=0)
    q = geometry_msgs.msg.Quaternion(x=cos(_theta*pi/180.0),
                                     y=0,
                                     z=0,
                                     w=sin(_theta*pi/180.0))
    pose = geometry_msgs.msg.Pose(position=p, orientation=q)
    pstamp = geometry_msgs.msg.PoseStamped(pose=pose)
    goal = move_base_msgs.msg.MoveBaseGoal(target_pose=pstamp)
    client.send_goal(goal)

def q():
    client.cancel_goal()

def get_tf():
    try:
        return listener.lookupTransform('leg_base_link', 'sample',
                                        rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        return (None, None)

def run():
    (trans, rot) = get_tf()
    if trans is None:
        return
    theta = atan2(trans[1], trans[0])
    if theta > pi/4.0:
        go_pos(0, 0, pi/4.0)
        time.sleep(2.0)
        (trans, rot) = get_tf()
        if trans is None:
            return
    elif theta < -pi/4.0:
        go_pos(0, 0, -pi/4.0)
        time.sleep(2.0)
        (trans, rot) = get_tf()
        if trans is None:
            return
    go_pos(trans[0]*1000, 0, theta)
