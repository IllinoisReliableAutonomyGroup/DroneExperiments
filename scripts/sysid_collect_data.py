#!/usr/bin/env python3
import rospy
import numpy as np
import argparse
import pickle5 as pickle
from DroneExperiments.sysid.ExcitationGenerator import gen_control_signal, gen_control_signal_pulse
from DroneExperiments.core.constants import dt
from DroneExperiments.ros.DroneInterface import Drone

parser = argparse.ArgumentParser(description="")
parser.add_argument('--ntraj', type=int, default=3, help='Number of trajectories.')
parser.add_argument('--T', type=float, default=10., help='Time horizon of the trajectory.')
parser.add_argument('--xyz', nargs='+', default=[1,0,0], type=int, help='Path to a directory for storing the log.')
parser.add_argument('--monitor', dest='monitor_mode', action='store_true')
parser.set_defaults(monitor_mode=False)
parser.add_argument('--pulse', dest='pulse_mode', action='store_true')
parser.set_defaults(pulse_mode=False)
parser.add_argument('--pulse_width', type=float, default=1., help='Width of the pulse.')
parser.add_argument('--pulse_height', type=float, default=1., help='Magnitude of the pulse.')
parser.add_argument('--save', type=str, default='data.pkl', help='filename to save the data.')
args = parser.parse_args()

def main():
    rospy.init_node('sysid', anonymous=True)
    drone = Drone()
    current_state = drone.state

    data = []

    rate = rospy.Rate(1. / dt)

    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    if args.monitor_mode:
        states = []
        controls = []
        t = 0.
        while not rospy.is_shutdown():
            msg = 't = %.3f s / %.3f s : '%(t, args.T)
            msg += 'current_position: (%.3f, %.3f, %.3f); current_velocity: (%.3f, %.3f, %.3f).'%tuple(current_state[:6])
            print(msg)

            states.append(current_state.copy())
            controls.append([0.,0,0])
            t += dt
            if t >= args.T:
                break
            rate.sleep()
        data.append([states, controls])
        with open(args.save, 'wb') as handle:
            pickle.dump([data, dt, args.T], handle, protocol=pickle.HIGHEST_PROTOCOL)
        return

    if args.pulse_mode:
        args.ntraj = 1

    for traj_id in range(args.ntraj):
        if rospy.is_shutdown():
            break
        rospy.loginfo('Trajectory %d'%traj_id)
        if args.pulse_mode:
            control_signal, start_position = gen_control_signal_pulse(xyz=args.xyz, T=args.T, pulse_width=args.pulse_width, pulse_height=args.pulse_height)
        else:
            control_signal, start_position = gen_control_signal(xyz=args.xyz, T=args.T)

        if not args.pulse_mode:
            # goto start position
            rospy.loginfo('GOTO (%f, %f, %f)'%(start_position[0], start_position[1], start_position[2]))
            drone.goto(start_position, dt=dt)
            rospy.loginfo('Reached (%f, %f, %f)'%(start_position[0], start_position[1], start_position[2]))

        # apply signal
        states = []
        controls = []
        rospy.loginfo('Start applying signals')
        rate.sleep()
        for t in range(control_signal.shape[0]):
            if rospy.is_shutdown():
                break
            a = control_signal[t]
            states.append(current_state.copy())
            controls.append(a)
            drone.set_acc(a[0], a[1], a[2], 0.)
            # print(drone.state)
            # now = rospy.get_rostime().to_sec()
            # print(now, drone.latest, drone.state[0])
            # drone.set_vel(a[0], a[1], a[2])
            rate.sleep()
        rospy.loginfo('Finished applying signal')
        data.append([states, controls])

        # stop
        rospy.loginfo('Start braking')
        drone.brake(dt=dt)
        rospy.loginfo('Finished braking')

    with open(args.save, 'wb') as handle:
        pickle.dump([data, dt, args.T], handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    main()
