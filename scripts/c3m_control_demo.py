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
import time
import os

parser = argparse.ArgumentParser(description="")
parser.add_argument('--initerror', type=float, default=0., help='error in initial position.')
parser.add_argument('--ulimit', type=float, default=2., help='max control cmd.')
parser.add_argument('--controller', type=str, help='filename to the pytorch model file.')
parser.add_argument('--ref', type=str, default='traj_ref.pkl', help='filename to the reference trajectories.')
parser.add_argument('--save', type=str, help='filename to save the data.')
args = parser.parse_args()

def main():
    rospy.init_node('c3m_tracking', anonymous=True)
    drone = Drone()
    current_state = drone.state

    controller = get_controller_wrapper(args.controller)

    with open(args.ref, 'rb') as handle:
        ref = pickle.load(handle)

    if not args.save:
        ref_name = os.path.basename(args.ref)[:-4]
        controller_name = os.path.basename(args.controller)[:-4]
        args.save = '../data/c3m_tracking_' + ref_name + '_' + controller_name + '_' + str(time.time()) + '.pkl'
    print('Will save to ' + args.save)

    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    # goto start position
    start_position = ref['start_position']
    start_position[0] += args.initerror * (np.random.rand() - 0.5)
    start_position[1] += args.initerror * (np.random.rand() - 0.5)
    rospy.loginfo('GOTO (%f, %f, %f)'%(start_position[0], start_position[1], start_position[2]))
    drone.goto(start_position, dt=dt)
    rospy.loginfo('Reached (%f, %f, %f)'%(start_position[0], start_position[1], start_position[2]))

    # start tracking
    xref = ref['xref']
    uref = ref['uref']
    data = []
    refs = []
    refs.append([xref, uref])
    ref_ids = []
    xstars = []
    ustars = []
    states = []
    controls = []
    rospy.loginfo('Start tracking')
    for t in range(xref.shape[0]):
        if rospy.is_shutdown():
            break
        xstar = xref[t, :]
        ustar = uref[t, :]
        xstars.append(xstar.copy())
        ustars.append(ustar.copy())
        # t1 = time.time()
        u = controller(current_state, xstar, ustar)
        # print('%.3fs'%(time.time() - t1))
        u = np.clip(u, -1. * args.ulimit, args.ulimit)
        # u = ustar
        states.append(current_state.copy())
        controls.append(u.copy())
        ref_ids.append(len(refs) - 1)
        drone.set_acc(u[0], u[1], u[2], 0.)
        rate.sleep()
    rospy.loginfo('Finished tracking')
    data.append([np.array(d) for d in [states, controls, ref_ids, xstars, ustars]])

    # stop
    rospy.loginfo('Start braking')
    drone.brake(dt=dt)
    rospy.loginfo('Finished braking')

    with open(args.save, 'wb') as handle:
        pickle.dump([data, dt, refs], handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    main()
