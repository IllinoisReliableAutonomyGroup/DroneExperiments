#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)



import rospy
import numpy as np
import pickle5 as pickle
import argparse
from pymavlink import mavutil
from DroneExperiments.ros.DroneInterface import Drone
from DroneExperiments.core.constants import dt
from DroneExperiments.vision.detect_aruco import detect_aruco, get_camera, release_camera
from DroneExperiments.vision.cam2drone import get_T_DC
from DroneExperiments.utils import transformations
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose
from DroneExperiments.utils.transformations import quaternion_from_matrix, euler_from_quaternion

from functools import partial
import time
import os
from std_msgs.msg import Bool

parser = argparse.ArgumentParser(description="")
parser.add_argument('--ulimit', type=float, default=2., help='max control cmd.')
parser.add_argument('--controller', type=str, help='filename to the pytorch model file.')
parser.add_argument('--save', type=str, help='filename to save the data.')
parser.add_argument('--oneshot', dest='oneshot', action='store_true')
parser.add_argument('--markersize', type=float, default=172, help='Size of the ArUco marker in mm.')
parser.set_defaults(oneshot=False)
args = parser.parse_args()

# new_xref = None

# def new_target_callback(msg):
#     global new_xref

#     if msg.data:
#         new_xref = 1

def main():
    # global new_xref


    # rospy.init_node('c3m_landing', anonymous=True)

    drone = Drone()
    # current_state = drone.state

    camera = get_camera()

    # Create a mavlink serial instance
    master = mavutil.mavlink_connection('udp:127.0.0.1:10101')
    # Make sure the connection is valid
    master.wait_heartbeat()

    T_DC = get_T_DC()

    rate = rospy.Rate(1. / dt)

    # plan a trajectory
    # rospy.Subscriber('/landing_vision_target', Bool, new_target_callback, queue_size=1)

    
    Ts = []
    while not rospy.is_shutdown() and len(Ts) == 0: 
        Ts, ids = detect_aruco(camera, visualize=False)
        print("enter loop1")
        rate.sleep()
    
    rospy.loginfo('Start tracking')
    print("enter loiter mode")
    drone.loiter_mode()
    

    while not rospy.is_shutdown():
        print("enter 2nd loop")
        Ts, ids = detect_aruco(camera, marker_size=args.markersize, visualize=True)

        pose_C = Ts[ids.index(args.targetid)]
        pose_D = T_DC.dot(pose_C)
        pose = np.linalg.inv(pose_D)
        translation = pose[:3, 3]
        q = quaternion_from_matrix(pose)

        print("t:", translation)
        print("R:", np.array(euler_from_quaternion(q, 'sxyz')) / np.pi  * 180.)

        x, y, z = translation
        qx, qy, qz, qw = q
        time_usec = int(time.time() * 1e6)
        master.mav.att_pos_mocap_send(time_usec, [qw, qy, qx, -qz], y, x, -z)

    release_camera(camera)

    

    print("Done")



if __name__ == '__main__':
    main()
