#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)
import time

import rospy
import argparse
from DroneExperiments.ros.DroneInterface import Drone
from DroneExperiments.core.constants import dt

parser = argparse.ArgumentParser(description="")
parser.add_argument('--move', type=str, default='vy', help='direction to move.')
args = parser.parse_args()

def main():
    rospy.init_node('test_movement', anonymous=True)
    drone = Drone()

    rate = rospy.Rate(1. / dt)
    time_interval = 1.2
    totalcount=time_interval/dt
    interval_count = 100
    totaltimes = 6
    currtime = 0
    count = 0 
    move_left = True
    print(totalcount)
    print(rate)


    rospy.loginfo('Start tracking')
    drone.guided_mode()

    drone.goto([3.0, 2.8, 1.5])
    time.sleep(3)

    while not rospy.is_shutdown():
        #        drone.set_acc(0.0, -7.0, 0.0, 0.0)
        #currtime += 1

        # if move_left:
        #     drone.set_acc(0.0, 7.0, 0.0, 0.0)
        #     count += 1
        #     if count >= interval_count:
        #         count = 0 
        #         move_left = False
        #         currtime += 1
        # else:
        #     drone.set_acc(0.0, -7.0, 0.0, 0.0)
        #     count += 1
        #     if count >= interval_count:
        #         count = 0 
        #         move_left = True
        #         currtime += 1  

        # print("moveleft:",move_left, currtime, count)

        if move_left:
            drone.set_vel(0, -1, 0)
            count += 1
            if count >= interval_count:
                count = 0 
                move_left = False
                currtime += 1
        else:
            drone.set_vel(0, 1, 0)
            count += 1
            if count >= interval_count:
                count = 0 
                move_left = True
                currtime += 1

        if currtime >= totaltimes:
            drone.set_vel(0, 0, 0)
            break

        #if currtime >= totaltimes:
        #    drone.set_acc(0.0, 0.0, 0.0, 0.0)
        #    break
        rate.sleep()
    
    rospy.loginfo('Finished tracking')
    for i in range(10):
        drone.disarm()
        rate.sleep()

    drone.loiter_mode()

if __name__ == '__main__':
    main()
