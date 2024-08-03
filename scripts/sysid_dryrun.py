#!/usr/bin/env python
import numpy as np
import argparse

from DroneExperiments.sysid.ExcitationGenerator import gen_control_signal, gen_control_signal_pulse

parser = argparse.ArgumentParser(description="")
parser.add_argument('--ntraj', type=int, default=10, help='Number of trajectories.')
parser.add_argument('--xyz', nargs='+', default=[1,1,1], type=int, help='Path to a directory for storing the log.')
parser.add_argument('--T', type=float, default=10., help='Number of trajectories.')
parser.add_argument('--pulse', dest='pulse_mode', action='store_true')
parser.set_defaults(pulse_mode=False)
parser.add_argument('--pulse_width', type=float, default=1., help='Width of the pulse.')
parser.add_argument('--pulse_height', type=float, default=1., help='Magnitude of the pulse.')
args = parser.parse_args()

def main():
    if args.pulse_mode:
        args.ntraj = 1
    for traj_id in range(args.ntraj):
        if args.pulse_mode:
            control_signal, start_position = gen_control_signal_pulse(xyz=args.xyz, T=args.T, pulse_width=args.pulse_width, pulse_height=args.pulse_height, vis=True)
        else:
            control_signal, start_position = gen_control_signal(xyz=args.xyz, T=args.T, vis=True)

if __name__ == '__main__':
    main()
