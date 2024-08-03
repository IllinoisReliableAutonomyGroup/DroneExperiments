#!/usr/bin/env python3
import numpy as np
import numpy.linalg as la
import rospy
import argparse
from DroneExperiments.ros.DroneInterface import Drone
from DroneExperiments.core.constants import dt
import time

np.random.seed(1024)

#top left 3.12153125, 3.056509765625,
#top right 3.130737548828125, -2.348701904296875
#bot right -1.83276025390625, -2.392014892578125
#bot left -1.432434326171875, 3.25838427734375,

parser = argparse.ArgumentParser(description="")
parser.add_argument('--type', type=str, default='vy', help='direction to move.')
parser.add_argument('--vel', type=float, default=-1.0, help='Vel of Drone.')
parser.add_argument('--displacement', type=float, default=3.0, help='Displacement of Drone.')
args = parser.parse_args()

def main():
    rospy.init_node('test_movement', anonymous=True)

    max_x = 3.5
    min_y = -0.8

    drone = Drone()
    rate = rospy.Rate(1. / dt)

    if args.type == 'vy':
        vy = args.vel
        vx = 0.0
    elif args.type == 'vx':
        vx = args.vel
        vy = 0.0
    else:
        print("Error setting velocity type")
        return
    print("vx vy:",vx,vy)
    
    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    rospy.loginfo('Start tracking')
    drone.guided_mode()
    drone.goto(np.array([2.69, 2.56, 1.0]))
    time.sleep(1)

    initial_state = np.array(drone.state[:3])

    while not rospy.is_shutdown():
    
        drone.set_vel(vx, vy, 0.0)
        print("set vel:",vx,vy)

        current_state = np.array(drone.state[:3])
        if current_state[0] >= max_x or current_state[1]<= min_y:
            drone.set_vel(0.0,0.0 ,0.0)
            break

        rate.sleep()

    current_state = np.array(drone.state[:3])
    drone.goto(current_state)
    
    rospy.loginfo('Finished tracking')
    for i in range(10):
        drone.disarm()
        rate.sleep()

    drone.loiter_mode()

if __name__ == '__main__':
    main()
