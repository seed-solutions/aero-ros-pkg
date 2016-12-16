#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import *

radius = 0.076
Radius = 0.2974535
max_velocity = 15 #rpm
wheel_names = ['can_front_l_wheel', 'can_front_r_wheel', 'can_rear_l_wheel', 'can_rear_r_wheel']

def msg2cmd(msg):
    # m/s -> rpm
    v = msg.linear.x / (0.10472 * radius)
    if abs(msg.angular.z) < 0.01: # translate
        print('translate')
        if v < 0:
            v = max(v, -max_velocity)
        else:
            v = min(v, max_velocity)
        return [v, -v, v, -v]
    else:
        # rad/s -> rpm
        w = 9.54930 * msg.angular.z
        w = math.sqrt(2) * Radius * w / radius
        if abs(msg.linear.x) < 0.01: # rotate
            print('rotate')
            if w < 0:
                w = max(w, -max_velocity)
            else:
                w = min(w, max_velocity)
            return [-w, -w, -w, -w]
        else: # translate + rotate
            print('drift')
            # w = vt * 0.5, v = V - vt
            V = v + 2 * abs(w)
            if V < 0:
                V = -max(V, -max_velocity)
            else:
                V = min(V, max_velocity)
            # when V neq V, move slower than actual command
            # a * w = vt * 0.5, a * v  = V - vt
            a = V / (3 * v)
            u = a * v
            if w < 0:
                if v < 0:
                    return [-u, V, -u, V]
                else:
                    return [V, -u, V, -u]
            else:
                if v < 0:
                    return [-V, u, -V, u]
                else:
                    return [u, -V, u, -V]
