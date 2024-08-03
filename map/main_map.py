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
import threading
exit_event = threading.Event()


def signal_handler(signum, frame):
    exit_event.set()

from map import markerDB

from functools import partial
import time
import os

parser = argparse.ArgumentParser(description="")
parser.add_argument('--save', type=str, help='filename to save the data.')
parser.add_argument('--targetid', type=int, default=14, help='ArUco ID of the target.')
parser.add_argument('--markersize', type=float, default=59, help='Size of the ArUco marker in mm.')
args = parser.parse_args()

class main():
    def __init__(self):
        pass
    

class mavCOM(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.time = 0
        self.pktSent = 0
        self.target_system = 1
        self.origin_lat = -35.363261
        self.origin_lon = 149.165230
        self.origin_alt = 0
        self.posDelta = (0, 0, 0)
        self.rotDelta = (0, 0, 0)   
        self.device = 'udp:127.0.0.1:10101'
        self.conn = None
        self.heartbeatTimestamp = time.time()
        self.lock = threading.Lock()
        self.pos = (0,0,0)
        self.rot = (0,0,0)
        self.translation = [0, 0, 0]
        self.quaternion = [0, 0, 0, 0]
        self.reset_counter = 0


    def run(self):
        # Start mavlink connection
        try:
            # Create a mavlink serial instance
            self.conn = mavutil.mavlink_connection(self.device)
        except Exception as msg:
            print("Failed to start mavlink connection on %s: %s" %
                    (self.device, msg,))
            raise


        # Make sure the connection is valid
        self.conn.wait_heartbeat()

        # wait for the heartbeat msg to find the system ID. Need to exit from here too
        # We are sending a heartbeat signal too, to allow ardupilot to init the comms channel
        while True:
            self.sendHeartbeatAndEKFOrigin()
            if self.conn.wait_heartbeat(timeout=0.5) != None:
                # Got a hearbeart, go to next loop
                self.goodToSend = True
                break
            if exit_event.is_set():
                return

        print("Got Heartbeat from APM (system %u component %u)" %
                (self.conn.target_system, self.conn.target_system))
        # self.send_msg_to_gcs("Starting")

        while True:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            # loop at 20 Hz
            time.sleep(0.05)
            self.sendPos()
            # self.sendSpeed()
            #self.sendPosDelta()
            self.sendHeartbeatAndEKFOrigin()
            if exit_event.is_set():
                # self.send_msg_to_gcs("Stopping")
                return

    def set_default_global_origin(self):
        current_time_us = int(round(time.time() * 1000000))
        self.conn.mav.set_gps_global_origin_send(self.target_system,
                                                 int(self.origin_lat*1.0e7),
                                                 int(self.origin_lon*1.0e7),
                                                 int(self.origin_alt*1.0e3),
                                                 current_time_us)
    def sendHeartbeatAndEKFOrigin(self):
            # send heartbeat and EKF origin messages if more than 1 sec since last message
            if (self.heartbeatTimestamp + 1) < time.time():
                self.conn.mav.heartbeat_send(2,
                                            0,
                                            0,
                                            0,
                                            0)
                # MAV_QUADROTOR = 2
                # MAV_AUTOPILOT_GENERIC = 0

                self.set_default_global_origin()
                self.heartbeatTimestamp = time.time()

    def sendPos(self):
        # Send a vision pos estimate
        # https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
        # if self.getTimestamp() > 0:
        # print("run sendPos")
        if self.goodToSend:
            current_time_us = int(round(time.time() * 1000000))
            # estimate error - approx 0.01m in pos and 0.5deg in angle
            cov_pose = 0.01
            cov_twist = 0.5
            covariance = np.array([cov_pose, 0, 0, 0, 0, 0,
                                      cov_pose, 0, 0, 0, 0,
                                      cov_pose, 0, 0, 0,
                                      cov_twist, 0, 0,
                                      cov_twist, 0,
                                      cov_twist])
            with self.lock:
                self.conn.mav.att_pos_mocap_send(
                    current_time_us, 
                    [self.quaternion[3], self.quaternion[1], self.quaternion[0], -self.quaternion[2]], 
                    self.translation[1], self.translation[0], -self.translation[2])
                self.pktSent += 1


    def set_default_global_origin(self):
        current_time_us = int(round(time.time() * 1000000))
        self.conn.mav.set_gps_global_origin_send(self.target_system,
                                                 int(self.origin_lat*1.0e7),
                                                 int(self.origin_lon*1.0e7),
                                                 int(self.origin_alt*1.0e3),
                                                 current_time_us)
        
    def updateData(self, t, newTrans, newQuat):
        print("Run update Data")
        with self.lock:
            self.translation = newTrans
            self.quaternion = newQuat
            self.time = t



if __name__ == '__main__':
    rospy.init_node('landing_vision_module', anonymous=True)
    drone = Drone()
    # current_state = drone.state

    # # pub = rospy.Publisher('/landing_vision_target', Vector3, queue_size=1)
    camera = get_camera()
    markerPlacement = markerDB()
    rate = rospy.Rate(1. / dt)

    
    # Start MAVLink comms thread
    print("start mavlink com")
    ConnectionMavlink = mavCOM()
    ConnectionMavlink.start()
    print("outside of loop")

    Ts = []
    first = True
    while not rospy.is_shutdown():
        Ts, ids = detect_aruco(camera, marker_size=args.markersize, visualize=False)
        
        if len(Ts) == 0: 
            rate.sleep()
        else:
            if first:
                rospy.loginfo('Start tracking')
                drone.loiter_mode()

            time_usec, new_translation, new_q = markerPlacement.getPose(Ts, ids)
            # x, y, z = translation
            # qx, qy, qz, qw = q
            # print("detected")
            timestamp = int(round(time.time() * 1000000)) + 50
            # Create and send MAVLink packet
            # new_translation = [0, 0, 0]
            # new_q = [0,0,0,0]
            ConnectionMavlink.updateData(timestamp, new_translation, new_q)
            print("translation",new_translation)


        # print("Exited loop")
        if exit_event.is_set():
            break    

    release_camera(camera)
    print("Done")