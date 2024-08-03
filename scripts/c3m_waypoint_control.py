#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
import numpy as np
import pickle5 as pickle
import argparse
from DroneExperiments.core.constants import dt
from DroneExperiments.ros.DroneInterface import Drone
from DroneExperiments.C3M.models.model_navio2_numpy import get_controller_wrapper
from DroneExperiments.core.refs.gen_xref_uref_from_xyzy import gen_xref_uref_from_xyzy
from functools import partial
import time
import os

parser = argparse.ArgumentParser(description="")
parser.add_argument('--controller_xy', type=str, help='filename to the pytorch model file.')
parser.add_argument('--wp_list', type=str, help='filename to waypoint list.')
parser.add_argument('--wp_interval', type=float, help='time interval for each wp.')
parser.add_argument('--wp_start', type=int, help='id of the first wp.')
parser.add_argument('--wp_end', type=int, help='id of the last wp.')
parser.add_argument('--save', type=str, help='filename to save the data.')
parser.add_argument('--lookat', nargs='+', default=[], type=float, help='The point to look at.')
args = parser.parse_args()

def main():
    rospy.init_node('c3m_waypoints', anonymous=True)
    drone = Drone()
    current_state = drone.state

    controller = get_controller_wrapper(args.controller_xy)

    if not args.save:
        controller_xy_name = os.path.basename(args.controller_xy)[:-4]
        wp_list_name = os.path.basename(args.wp_list)[:-4]
        args.save = '../data/c3m_waypoints_lookat_%.3f_%.3f_'%(args.lookat[0], args.lookat[1]) + controller_xy_name + '_' + str(args.wp_interval) + '_' + wp_list_name + '_' + str(time.time()) + '.pkl'
    print('Will save to ' + args.save)

    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    assert len(args.lookat) == 2

    # start tracking
    data = []
    refs = [[None, None], ]
    ref_ids = []
    xstars = []
    ustars = []
    states = []
    controls = []

    rospy.loginfo('Started')

    with open(args.wp_list, 'rb') as handle:
        wp_list = pickle.load(handle)

    print('len(wp_list) = %d'%len(wp_list))

    wp_init_interval = 10.

    for wp_id in range(args.wp_start, args.wp_end):
        if rospy.is_shutdown():
            break
        wp = wp_list[wp_id]
        print(wp_id, wp)
        position_hold_cnt = 0
        while not rospy.is_shutdown():
            position_hold_cnt += 1
            interval = args.wp_interval if wp_id > args.wp_start else wp_init_interval
            if position_hold_cnt * dt > interval:
                break
            yaw = np.arctan2(wp[0] - args.lookat[0], -(wp[1] - args.lookat[1]))
            xstar = np.array(wp + [0, ] * 6 + [yaw, 0])
            ustar = np.array([0, ] * 4)
            xstars.append(xstar.copy())
            ustars.append(ustar.copy())
            u = controller(current_state, xstar, ustar)
            states.append(current_state.copy())
            controls.append(u.copy())
            ref_ids.append(len(refs) - 1)
            drone.set_acc_yaw(u[0], u[1], u[2], yaw)
            rate.sleep()

    rospy.loginfo('Finished')

    data.append([np.array(d) for d in [states, controls, ref_ids, xstars, ustars]])
    with open(args.save, 'wb') as handle:
        pickle.dump([data, dt, refs], handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    main()