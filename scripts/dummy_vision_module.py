#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)
import time

from pymavlink import mavutil
import rospy
from DroneExperiments.ros.DroneInterface import Drone
from DroneExperiments.core.constants import dt

def main():
    rospy.init_node('dummy_vision_module', anonymous=True)

    # Create a mavlink serial instance
    master = mavutil.mavlink_connection('udp:127.0.0.1:10101')
    # Make sure the connection is valid
    master.wait_heartbeat()

    drone = Drone()
    current_state = drone.state

    dt = 0.2
    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    while not rospy.is_shutdown():
        x, y, z = current_state[:3]
        qx, qy, qz, qw = current_state[11:15] # x, y, z, w
        time_usec = int(time.time() * 1e6)
        master.mav.att_pos_mocap_send(time_usec, [qw, qy, qx, -qz], y, x, -z)
        rate.sleep()

if __name__ == '__main__':
    main()
