#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
import numpy as np
import pickle5 as pickle
import argparse
from DroneExperiments.ros.DroneInterface import Drone
from DroneExperiments.core.constants import dt
from DroneExperiments.vision.detect_aruco import detect_aruco, get_camera, release_camera
from DroneExperiments.vision.cam2drone import get_T_DC
from DroneExperiments.utils import transformations
from geometry_msgs.msg import Vector3, Point, Quaternion, Pose

from functools import partial
import time
import os

parser = argparse.ArgumentParser(description="")
parser.add_argument('--save', type=str, help='filename to save the data.')
parser.add_argument('--targetid', type=int, default=11, help='ArUco ID of the target.')
parser.add_argument('--markersize', type=float, default=59, help='Size of the ArUco marker in mm.')
args = parser.parse_args()

def main():
    rospy.init_node('landing_vision_module', anonymous=True)
    drone = Drone()
    current_state = drone.state

    pub = rospy.Publisher('/landing_vision_target', Vector3, queue_size=1)
    camera = get_camera()

    pub_Pose = rospy.Publisher('/mavros_extras/vision_pose/pose', Pose, queue_size=1)

    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    T_DC = get_T_DC()

    data = []
    cnt = 0
    while not rospy.is_shutdown():
        snap_state = current_state.copy()
        Ts, ids = detect_aruco(camera, marker_size=args.markersize, save=args.save+'.aruco_%d.jpg'%cnt)

        if len(Ts) > 0:
            pose_C = Ts[ids.index(args.targetid)]
            q = snap_state[11:15] # x, y, z, w
            T_WD = transformations.quaternion_matrix(q)
            T_WD[:3, 3] = snap_state[:3]
            pose_D = T_DC.dot(pose_C)
            pose_W = T_WD.dot(pose_D)
            target = pose_W[:3, 3]
            msg = Vector3()
            msg.x = target[0]
            msg.y = target[1]
            msg.z = target[2]
            rospy.loginfo('Found new target: [%.3f, %.3f, %.3f]'%(target[0], target[1], target[2]))
            pub.publish(msg)
            data.append([pose_C, T_WD, snap_state, pose_D, pose_W, cnt])
            cnt += 1


        # if len(Ts) > 0 and args.targetid in ids:
        #     pose_C = Ts[ids.index(args.targetid)]
        #     q = snap_state[11:15] # x, y, z, w
        #     T_WD = transformations.quaternion_matrix(q)
        #     T_WD[:3, 3] = snap_state[:3]
        #     pose_D = T_DC.dot(pose_C)
        #     pose_W = T_WD.dot(pose_D)
        #     target = pose_W[:3, 3]
        #     msg = Vector3()
        #     msg.x = target[0]
        #     msg.y = target[1]
        #     msg.z = target[2]
        #     rospy.loginfo('Found new target: [%.3f, %.3f, %.3f]'%(target[0], target[1], target[2]))
        #     pub.publish(msg)
        #     data.append([pose_C, T_WD, snap_state, pose_D, pose_W, cnt])
        #     cnt += 1
    release_camera(camera)

    if args.save:
        with open(args.save, 'wb') as handle:
            pickle.dump(data, handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    main()
