#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
import numpy as np
import importlib
import pickle5 as pickle
import argparse
from DroneExperiments.core.constants import dt
from DroneExperiments.ros.DroneInterface import Drone, distance
from DroneExperiments.C3M.models.model_navio2_numpy import get_controller_wrapper
from DroneExperiments.core.refs.gen_xref_uref_from_xyz import gen_xref_uref_from_xyz
from functools import partial
from threading import Lock
import time
import os   

parser = argparse.ArgumentParser(description="")
parser.add_argument('--ulimit', type=float, default=2., help='max control cmd.')
parser.add_argument('--controller', type=str,default="controller_xy.pkl", help='filename to the pytorch model file.')
parser.add_argument('--save', type=str, help='filename to save the data.')
parser.add_argument('--oneshot', dest='oneshot', action='store_true')
parser.add_argument('--T', type=float, default=30., help='Time horizon of the curve.')
parser.add_argument('--type', type=str, default='lemniscate', help='Type of curve.')
parser.add_argument('--debug', type=bool, default=False, help='Visualization Preference.')
parser.add_argument('--gen_args', nargs='+', default=[15,2,0.5], type=float, help='Arguments to the curve generator.')
parser.set_defaults(oneshot=False)

args = parser.parse_args()

print("Starting Script")

curve_gen = importlib.import_module('DroneExperiments.core.refs.'+args.type)

new_xref = None
new_uref = None
new_ref_available = False
new_ref_lock = Lock()
current_state = None

def main():
    global new_xref, new_uref, new_ref_available, new_ref_lock, current_state

    rospy.init_node('c3m_drone', anonymous=True)
    rate = rospy.Rate(1. / dt)

    drone = Drone()
    current_state = drone.state
    controller = get_controller_wrapper(args.controller)

    if not args.save:
        controller_name = os.path.basename(args.controller)[:-4]
        args.save = 'datalog_'+ args.type +'_' + controller_name + '_' + str(time.time()) + '.pkl'
    print('Will save to ' + args.save)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    # Ready for tracking
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

    ##### Plan trajectory #####
    xyz = partial(curve_gen.xyz, args.gen_args)
    xref, uref, t = gen_xref_uref_from_xyz(xyz, args.T, dt, translate=False, debug=args.debug)

    xref_endstates = np.tile(xref[-1:], (20, 1))
    uref_endstates = np.zeros((20,3))
    
    with new_ref_lock:
        new_xref = np.concatenate((xref, xref_endstates), axis=0)
        new_uref = np.concatenate((uref, uref_endstates), axis=0)
        new_ref_available = True

    print("xref",type(new_xref),new_xref.shape)
    print("uref",type(new_uref),new_uref.shape)
    print("ref available? ",new_ref_available)

    ##### Start Tracking #####
    rospy.loginfo('Start tracking')
    drone.guided_mode()

    while not rospy.is_shutdown():
       
        with new_ref_lock:
            if new_ref_available:
                if not args.oneshot or xref is None:
                    xref = new_xref
                    uref = new_uref
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
        print("setting Accel: ",u)
        drone.set_acc(u[0], u[1], u[2], 0.)
        t += 1
        rate.sleep()

    rospy.loginfo('Finished tracking')

    for i in range(5):
        drone.disarm()
        rate.sleep()

    drone.loiter_mode()

    data.append([np.array(d) for d in [states, controls, ref_ids, xstars, ustars]])

    with open(args.save, 'wb') as handle:
        pickle.dump([data, dt, refs], handle, protocol=pickle.HIGHEST_PROTOCOL)
        print("saved")

if __name__ == '__main__':
    main()
