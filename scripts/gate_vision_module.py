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
from geometry_msgs.msg import PoseArray, Pose
from functools import partial
import time
import os

parser = argparse.ArgumentParser(description="")
parser.add_argument('-id','--gates_id', type=int, nargs='+', action='append', help='IDs of the ArUco markers.')
parser.add_argument('--markersize', type=float, default=59, help='Size of the ArUco marker in mm.')
args = parser.parse_args()

for mids in args.gates_id:
    assert len(mids) == 4

def main():
    rospy.init_node('gate_vision_module', anonymous=True)
    drone = Drone()
    current_state = drone.state

    pub = rospy.Publisher('/gate_vision_position', PoseArray, queue_size=1)
    camera = get_camera()

    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    T_DC = get_T_DC()

    while not rospy.is_shutdown():
        snap_state = current_state.copy()
        Ts, ids = detect_aruco(camera, marker_size=args.markersize)
        q = snap_state[11:15] # x, y, z, w
        T_WD = transformations.quaternion_matrix(q)
        T_WD[:3, 3] = snap_state[:3]

        msg = PoseArray()

        rospy.loginfo('-----------------------------')

        for gid, mids in enumerate(args.gates_id):
            if any([i not in ids for i in mids]):
                continue
            positions = []
            directions = []
            for i in mids:
                T = Ts[ids.index(i)]
                positions.append(T.dot([0, 0, 0, 1]))
                directions.append(T.dot([0, 0, 1, 1]))
            position = T_WD.dot(T_DC.dot(np.mean(np.array(positions), axis = 0)))
            direction = T_WD.dot(T_DC.dot(np.mean(np.array(directions), axis = 0)))
            direction = (direction - position)[:3]
            direction = direction / np.sqrt((direction**2).sum())

            pose = Pose()
            pose.position.x = position[0]
            pose.position.y = position[1]
            pose.position.z = position[2]
            pose.orientation.x = direction[0]
            pose.orientation.y = direction[1]
            pose.orientation.z = direction[2]
            pose.orientation.w = gid
            msg.poses.append(pose)

            rospy.loginfo('Found new gate: %s, %s'%(str(position[:3].tolist()), str(direction[:3].tolist())))

        pub.publish(msg)

    release_camera(camera)

if __name__ == '__main__':
    main()
