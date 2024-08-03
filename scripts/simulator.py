#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
# from matplotlib import animation
import argparse
import time
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import PositionTarget
from DroneExperiments.core.constants import dt, xy_Tc, z_Tc

state = np.array([0., 0, 0, 0, 0, 0, 0, 0, 0, 0])

def setpoint_raw_callback(data):
    global state
    ax_cmd = data.acceleration_or_force.y
    ay_cmd = -1.0 * data.acceleration_or_force.x
    az_cmd = data.acceleration_or_force.z
    yaw_rate = data.yaw_rate
    vx_cmd = data.velocity.y
    vy_cmd = -1.0 * data.velocity.x
    vz_cmd = data.velocity.z
    if ax_cmd == 0 and ay_cmd == 0 and az_cmd == 0:
        gain = 1.
        ax_cmd = (vx_cmd - state[3]) * gain
        ay_cmd = (vy_cmd - state[4]) * gain
        az_cmd = (vz_cmd - state[5]) * gain
    dot = np.array([state[3], state[4], state[5], state[6], state[7], state[8], 1 / xy_Tc * (ax_cmd - state[6]), 1 / xy_Tc * (ay_cmd - state[7]), 1 / z_Tc * (az_cmd - state[8]), yaw_rate])
    state += dot * dt
    # print('updated state using setpoint_raw')

def main():
    rospy.init_node('simulator', anonymous=True)
    pub_vicon = rospy.Publisher('/vicon_estimate', Float64MultiArray, queue_size=1)

    rospy.Subscriber('/mavros/setpoint_raw/local', PositionTarget, setpoint_raw_callback, queue_size=1)

    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        now = rospy.get_rostime().to_sec()
        msg.data = state[:9].tolist() + [now,]
        pub_vicon.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
