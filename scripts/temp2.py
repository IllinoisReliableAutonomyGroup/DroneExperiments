#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State

current_state = State()

def state_cb(state):
    global current_state
    current_state = state

rospy.init_node('set_guided_mode_node')

# Subscribers
state_sub = rospy.Subscriber('/mavros/state', State, state_cb)

# Wait for services
rospy.wait_for_service('/mavros/set_mode')
rospy.wait_for_service('/mavros/cmd/arming')

# Service proxies
set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

rate = rospy.Rate(10)  # 10 Hz

# Wait for FCU connection
while not rospy.is_shutdown() and not current_state.connected:
    rospy.loginfo("Waiting for FCU connection...")
    rate.sleep()

rospy.loginfo("FCU connected")

# Arm the vehicle
try:
    arming_response = arming_client(value=True)
    if arming_response.success:
        rospy.loginfo("Vehicle armed successfully")
    else:
        rospy.logerr("Failed to arm vehicle")
except rospy.ServiceException as e:
    rospy.logerr("Arming service call failed: %s" % e)

# Wait for the vehicle to be armed
while not rospy.is_shutdown() and not current_state.armed:
    rospy.loginfo("Waiting for vehicle to arm...")
    rate.sleep()

rospy.loginfo("Vehicle armed")

# Set mode to GUIDED
try:
    set_mode_response = set_mode_client(custom_mode='GUIDED')
    if set_mode_response.mode_sent:
        rospy.loginfo("GUIDED mode set successfully")
    else:
        rospy.logerr("Failed to set GUIDED mode")
except rospy.ServiceException as e:
    rospy.logerr("Set mode service call failed: %s" % e)

