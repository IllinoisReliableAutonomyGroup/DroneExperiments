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

def main():
    rospy.init_node('landing_vision_module', anonymous=True)
    drone = Drone()
    current_state = drone.state

    # pub = rospy.Publisher('/landing_vision_target', Vector3, queue_size=1)
    camera = get_camera()

    # # Create a mavlink serial instance
    # master = mavutil.mavlink_connection('udp:127.0.0.1:10101')
    # # Make sure the connection is valid
    # master.wait_heartbeat()

    # while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
    #     rate.sleep()

    T_DC = get_T_DC()

    data = []
    cnt = 0
    print("start loop")
    while not rospy.is_shutdown():
        # snap_state = current_state.copy()
        # # print("curt state",snap_state)

        Ts, ids = detect_aruco(camera, visualize=True)
        # if len(Ts) > 0:
        #     print(ids)

        if len(Ts) > 0:
            pose_C = Ts[0]
            pose_D = T_DC.dot(pose_C)
            pose = np.linalg.inv(pose_D)
            translation = pose[:3, 3]
            q = quaternion_from_matrix(pose)

            # print("t:", translation)
            # print("R:", np.array(euler_from_quaternion(q, 'sxyz')) / np.pi  * 180.)

            x, y, z = translation
            qx, qy, qz, qw = q
            time_usec = int(time.time() * 1e6)
            # master.mav.att_pos_mocap_send(time_usec, [qw, qy, qx, -qz], y, x, -z)

            # q = snap_state[11:15] # x, y, z, w
            # T_WD = transformations.quaternion_matrix(q)
            # T_WD[:3, 3] = snap_state[:3]
            # msg = Vector3()
            # msg.x = target[0]
            # msg.y = target[1]
            # msg.z = target[2]
            # rospy.loginfo('Found new target: [%.3f, %.3f, %.3f]'%(target[0], target[1], target[2]))
            # pub.publish(msg)
            # data.append([pose_C, T_WD, snap_state, pose_D, pose_W, cnt])
            # print("Pose_C:  ", pose_C)
            # print("Pose_W:  ", pose_W)
            # print("Pose_D:  ", pose_D)
            # print("translation:  ", translation)
            # print("Q:  ", q)
            print("Euler angles: ", np.array(euler_from_quaternion(q, 'sxyz')) / np.pi  * 180.)
            # print("Quaternion from snapstate", snap_state[11:15])
            # print("Calculated Quaternion",quaternion_from_matrix(Ts[0][:4,:4]))
    release_camera(camera)

    # if args.save:
    #     with open(args.save, 'wb') as handle:
    #         pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    main()