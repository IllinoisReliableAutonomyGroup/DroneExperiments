#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
import numpy as np
import pickle5 as pickle
import argparse
from DroneExperiments.core.constants import dt
from DroneExperiments.ros.DroneInterface import Drone, distance
from DroneExperiments.C3M.models.model_navio2_numpy import get_controller_wrapper
from DroneExperiments.core.refs.gen_xref_uref_from_xyz import gen_xref_uref_from_xyz
from DroneExperiments.core.refs import landing
from geometry_msgs.msg import Vector3
from functools import partial
from threading import Lock
import time
import os

parser = argparse.ArgumentParser(description="")
parser.add_argument('--ulimit', type=float, default=2., help='max control cmd.')
parser.add_argument('--controller', type=str, help='filename to the pytorch model file.')
parser.add_argument('--save', type=str, help='filename to save the data.')
parser.add_argument('--oneshot', dest='oneshot', action='store_true')
parser.set_defaults(oneshot=False)
args = parser.parse_args()

new_xref = None
new_uref = None
new_target = None
new_ref_available = False
new_ref_lock = Lock()
current_state = None

def new_target_callback(data):
    global new_xref, new_uref, new_ref_available, new_ref_lock, new_target, current_state
    target = np.array([data.x, data.y, data.z])
    snap_position = current_state[:3].copy()
    v1 = 1.0
    v2 = 1.0
    time_hover = 2.
    T = distance(snap_position[:2], target[:2]) / v1 + time_hover + distance(snap_position[2:3], target[2:3]) / v2 + 1.
    xyz = partial(landing.xyz, [snap_position[:3], target, v1, v2, time_hover])
    xref, uref, t = gen_xref_uref_from_xyz(xyz, T, dt, translate=False)
    with new_ref_lock:
        new_xref = xref
        new_uref = uref
        new_target = target
        new_ref_available = True

def main():
    global new_xref, new_uref, new_ref_available, new_ref_lock, new_target, current_state

    rospy.init_node('c3m_landing', anonymous=True)

    drone = Drone()
    current_state = drone.state


    controller = get_controller_wrapper(args.controller)

    if not args.save:
        controller_name = os.path.basename(args.controller)[:-4]
        args.save = '../data/c3m_landing_' + controller_name + '_' + str(time.time()) + '.pkl'
    print('Will save to ' + args.save)

    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    # plan a trajectory
    rospy.Subscriber('/landing_vision_target', Vector3, new_target_callback, queue_size=1)

    # start tracking
    refs = []
    data = []
    states = []
    xstars = []
    ustars = []
    controls = []
    ref_ids = []
    xref = None
    uref = None
    t = 0
    while not rospy.is_shutdown() and new_xref is None:
        rate.sleep()

    rospy.loginfo('Start tracking')

    drone.guided_mode()

    while not rospy.is_shutdown():
        if current_state[2] < 0.05:
            # landed
            break
        with new_ref_lock:
            if new_ref_available:
                if not args.oneshot or xref is None:
                    xref = new_xref
                    uref = new_uref
                    rospy.loginfo('Found new target: [%.3f, %.3f, %.3f]'%(new_target[0], new_target[1], new_target[2]))
                    new_ref_available = False
                    refs.append([xref, uref])
                    t = 0
        if t >= xref.shape[0]:
            break
        xstar = xref[t, :]
        ustar = uref[t, :]
        u = controller(current_state, xstar, ustar)
        u = np.clip(u, -1. * args.ulimit, args.ulimit)
        states.append(current_state.copy())
        xstars.append(xstar.copy())
        ustars.append(ustar.copy())
        controls.append(u.copy())
        ref_ids.append(len(refs) - 1)
        drone.set_acc(u[0], u[1], u[2], 0.)
        t += 1
        rate.sleep()

    rospy.loginfo('Finished tracking')

    for i in range(10):
        drone.disarm()
        rate.sleep()

    drone.loiter_mode()

    data.append([np.array(d) for d in [states, controls, ref_ids, xstars, ustars]])

    with open(args.save, 'wb') as handle:
        pickle.dump([data, dt, refs], handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    main()
