#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
import argparse
from DroneExperiments.ros.DroneInterface import Drone
from DroneExperiments.core.constants import dt

parser = argparse.ArgumentParser(description="")
parser.add_argument('--move', type=str, default='vy-', help='direction to move.')
args = parser.parse_args()

def main():
    rospy.init_node('test_movement', anonymous=True)
    drone = Drone()

    rate = rospy.Rate(1. / dt)

    rospy.loginfo('Started')
    while not rospy.is_shutdown():
        print(args.move)
        if args.move == 'vx':
            drone.set_vel(1, 0, 0)
        elif args.move == 'vx-':
            drone.set_vel(-1, 0, 0)
        elif args.move == 'vy':
            drone.set_vel(0, 1, 0)
        elif args.move == 'vy-':
            drone.set_vel(0, -1, 0)
        elif args.move == 'vz':
            drone.set_vel(0, 0, 1)
        elif args.move == 'vz-':
            drone.set_vel(0, 0, -1)
        elif args.move == 'ax':
            drone.set_acc(1, 0, 0, 0)
        elif args.move == 'ax-':
            drone.set_acc(-1, 0, 0, 0)
        elif args.move == 'ay':
            drone.set_acc(0, 1, 0, 0)
        elif args.move == 'ay-':
            drone.set_acc(0, -1, 0, 0)
        elif args.move == 'az':
            drone.set_acc(0, 0, 1, 0)
        elif args.move == 'az-':
            drone.set_acc(0, 0, -1, 0)
        rate.sleep()

if __name__ == '__main__':
    main()
