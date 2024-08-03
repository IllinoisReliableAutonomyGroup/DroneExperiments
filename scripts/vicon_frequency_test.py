#!/usr/bin/env python

''' 
Tests the affect of Vicon's position transmission frequency to Loiter mode stability
'''
import csv  
import rospy
from pymavlink import mavutil
import sys, os
import numpy as np
from std_msgs.msg import Float64MultiArray
import argparse

def main():
    rospy.init_node('vicon_bridge', anonymous=True)
    pub = rospy.Publisher('/vicon_estimate', Float64MultiArray, queue_size=1)

    # create a mavlink serial instance
    master = mavutil.mavlink_connection('udpin:0.0.0.0:10086')

    data = Float64MultiArray()

    data.data = [0, ] * (9 + 2 + 4 + 1)

    while not rospy.is_shutdown():
        msg = master.recv_match(blocking=False)
        if not msg:
            continue

        if msg.get_type() == 'LOCAL_POSITION_NED_COV':
            data.data[0] = msg.x / 1000.
            data.data[1] = msg.y / 1000.
            data.data[2] = msg.z / 1000.
            data.data[3] = msg.vx / 1000.
            data.data[4] = msg.vy / 1000.
            data.data[5] = msg.vz / 1000.
            data.data[6] = msg.ax / 1000.
            data.data[7] = msg.ay / 1000.
            data.data[8] = msg.az / 1000.

            # print("Coordinaes recieved")
            # use msg.covaricane to store the yaw and yaw_rate, and q
            offset = 100.
            data.data[9] = msg.covariance[0] - offset
            data.data[10] = msg.covariance[1] - offset

            data.data[11] = msg.covariance[2] - offset
            data.data[12] = msg.covariance[3] - offset
            data.data[13] = msg.covariance[4] - offset
            data.data[14] = msg.covariance[5] - offset

            now = rospy.get_rostime()
            now = now.to_sec()
            data.data[-1] = now
            pub.publish(data)

            print("Data",data)
        elif msg.get_type() == 'ATT_POS_MOCAP':
            pass

if __name__ == '__main__':
        
    parser = argparse.ArgumentParser()
    parser.add_argument("--tagSize", type=int, default=96,
                        help="Arucomarker size in mm")
    parser.add_argument("--camera", type=str, default="GenericUSB",
                        help="Camera profile in camera.yaml")
    parser.add_argument("--maxerror", type=int, default=400,
                        help="Maximum pose error to use, in n*E-8 units")
    parser.add_argument("--outfile", type=str, default="geo_test_results.csv",
                        help="Output tag data to this file")
    parser.add_argument(
        "--device", type=str, default='udp:127.0.0.1:10101', help="MAVLink connection string")
    parser.add_argument("--baud", type=int, default=115200,
                        help="MAVLink baud rate, if using serial")
    parser.add_argument("--source-system", type=int,
                        default=1, help="MAVLink Source system")
    parser.add_argument("--imageFolder", type=str, default="",
                        help="Save processed images to this folder")
    parser.add_argument("--video", type=int, default=0,
                        help="Output video to port, 0 to disable")
    parser.add_argument("--decimation", type=int,
                        default=2, help="Apriltag decimation")
    args = parser.parse_args()

    
    
    header = ['name', 'area', 'country_code2', 'country_code3'] 
    data = ['Afghanistan', 652090, 'AF', 'AFG']

    with open('countries.csv', 'w', encoding='UTF8') as f:
        writer = csv.writer(f)

        # write the header
        writer.writerow(header)

        # write the data
        writer.writerow(data)


    main()