#!/usr/bin/env python

from math import *
import time

import roslib
import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {h_limit_max}, {h_limit_min}, {s_limit_max}, {s_limit_min}, {i_limit_max}, {i_limit_min}".format(**config))

client = dynamic_reconfigure.client.Client("/stereo/hsi_color_filter/hsi_filter",
                                           timeout=30,
                                           config_callback=callback)
def init():
    rospy.init_node('dynamic_reconfigure_py')

def set_hsi(hM, hm, sM, sm, iM, im):
    client.update_configuration({"h_limit_max":hM, "h_limit_min":hm,
                                 "s_limit_max":sM, "s_limit_min":sm,
                                 "i_limit_max":iM, "i_limit_min":im})

