#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
import numpy as np
import pickle5 as pickle
import argparse
from pymavlink import mavutil

from DroneExperiments.core.constants import dt
from DroneExperiments.vision.detect_aruco import detect_aruco, get_camera, release_camera
from std_msgs.msg import Bool

import time
import os


def main():
    rospy.init_node('landing_vision_module', anonymous=True)
    pub = rospy.Publisher('/landing_vision_target', Bool, queue_size=1)
    camera = get_camera()

    data = []
    cnt = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        Ts, ids = detect_aruco(camera, visualize=False)

        if len(Ts) > 0:
            spotted = True
            pub.publish(spotted)
            print("ID IS ",ids)
            rate.sleep()
            
    release_camera(camera)

if __name__ == '__main__':
    main()