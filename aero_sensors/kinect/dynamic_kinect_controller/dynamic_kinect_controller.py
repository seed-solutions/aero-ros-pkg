#!/usr/bin/env python

from struct import *

import rospy
import time
import array
import math
import curses
import threading
from seed_command import SeedCommand
from std_msgs.msg import *

ID = 1
seed = SeedCommand("/dev/tilt_motor", 57413)
mutex = threading.Lock()

SPD = 1000

def command_subscriber(data):

    pulse = data.data * 90.62083333333334

    mutex.acquire()
    try:
        seed.SEED_DX_MOV(ID, SPD, int(round(pulse)))
    finally:
        mutex.release()

def speed_subscriber(data):

    global SPD

    SPD = data.data

class position_callback(threading.Thread):

    def __init__(self):

        super(position_callback, self).__init__()

        self.publisher = rospy.Publisher('/kinect_controller/state', Float32, queue_size=10)

    def run(self):

        while not rospy.is_shutdown():

            mutex.acquire()
            try:
                seed.SEED_DX_Get_POS(ID)

                pulse = seed.SEED_DX_Read()

                pulse = int(pulse[-4:-2] + pulse[-6:-4], 16) * 0.011034990114488023

                self.publisher.publish(Float32(pulse))
            finally:
                mutex.release()

                time.sleep(0.1)

#----------- main -----------------#
if __name__ == "__main__":

    seed.SEED_DX_SRV(ID, 1)

    time.sleep(0.5)

    rospy.init_node('dynamic_kinect_controller')

    command_subscriber = rospy.Subscriber('/kinect_controller/command', Float32, command_subscriber)

    speed_subscriber = rospy.Subscriber('/kinect_controller/speed', Int32, speed_subscriber)

    pos_callback = position_callback()

    pos_callback.start()

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
