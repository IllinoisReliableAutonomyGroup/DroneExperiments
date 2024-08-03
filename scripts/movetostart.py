#!/usr/bin/env python3
import numpy as np
import numpy.linalg as la
import rospy
import argparse
from DroneExperiments.ros.DroneInterface import Drone
from DroneExperiments.core.constants import dt
import time

np.random.seed(1024)

parser = argparse.ArgumentParser(description="")
parser.add_argument('--type', type=str, default='vy', help='direction to move.')
parser.add_argument('--vel', type=float, default=-3.0, help='Vel of Drone.')
parser.add_argument('--displacement', type=float, default=2.8, help='Displacement of Drone.')
args = parser.parse_args()

def main():
    rospy.init_node('test_movement', anonymous=True)

    drone = Drone()
    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    # 3.1320400390625, 2.34102197265625, 1.3
    # 3.36573046875, -2.170001708984375,
    # Distance y direction 4.3

    rospy.loginfo('Start tracking')
    drone.guided_mode()
    
    # Used for P2
    drone.goto(np.array([2.69, 2.56, 1.0]))
    
    #used for p0 p1
    #drone.goto(np.array([3.1, 2.8, 1.3]))
    time.sleep(1)
    rospy.loginfo('Finished tracking')
    for i in range(10):
        drone.disarm()
        rate.sleep()

    drone.loiter_mode()

if __name__ == '__main__':
    main()
