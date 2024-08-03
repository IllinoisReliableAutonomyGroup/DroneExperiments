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

    # leader at 
    # -2.141596923828125, 0.656233642578125

    # Close range
    # -1.1605, 1.1844, 1.0
    # -1.1675, 0.2816, 1.0

    # Mid range
    # 0.5470, 2.4358, 1.0
    # 0.6221, -0.4732, 1.0

    # Far range
    # 3.1657, 3.5643, 1.0
    # 3.1455, -1.6901, 1.0


    iteration_count = 3
    current_iteration = 0

    # # Close range
    # left_target = np.array([-1.0, 2.94, 1.0])
    # right_target = np.array([-1.0, -0.97, 1.0])

    # # Mid range
    # left_target = np.array([0.55, 2.94, 1.0])
    # right_target = np.array([0.55, -0.97, 1.0])

    # Far range
    left_target = np.array([2.832, 2.94, 1.0])
    right_target = np.array([2.832, -0.97, 1.0])

    # Range 1
    # left_target = np.array([-1.379, 2.94, 1.0])
    # right_target = np.array([-1.379, -0.97, 1.0])

    # Mid range
    # left_target = np.array([0.55, 2.94, 1.0])
    # right_target = np.array([0.55, -0.97, 1.0])

    # # Far range
    # left_target = np.array([3.16, 2.94, 1.0])
    # right_target = np.array([3.16, -0.97, 1.0])


    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()


    rospy.loginfo('Start tracking')
    drone.guided_mode()

    while not rospy.is_shutdown():
        print("Iteration #",current_iteration)

        if current_iteration % 2 == 0:
            drone.goto(position=left_target,tolerance = 0.3)
            print("Target:",left_target)
        else: 
            drone.goto(position=right_target, tolerance = 0.3)
            print("Target:",right_target)

        current_iteration +=1
        if current_iteration > iteration_count:
            break

    rospy.loginfo('Finished tracking')
    for i in range(10):
        drone.disarm()
        rate.sleep()

    drone.loiter_mode()

if __name__ == '__main__':
    main()
