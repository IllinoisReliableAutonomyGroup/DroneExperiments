#!/usr/bin/env python
import numpy as np
import argparse
import importlib
import pickle5 as pickle
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
sys.path.append('/Users/yang/')
from DroneExperiments.core.refs.gen_xref_uref_from_xyz import gen_xref_uref_from_xyz
from DroneExperiments.core.constants import dt, x_Tc,y_Tc, z_Tc
from functools import partial

parser = argparse.ArgumentParser(description="")
parser.add_argument('--T', type=float, default=30., help='Time horizon of the curve.')
parser.add_argument('--save', type=str, help='Filename to save the data.')
parser.add_argument('--type', type=str, default='lemniscate', help='Type of curve.')
parser.add_argument('--vis', dest='vis', action='store_true', help='Visualize the curve.')
parser.set_defaults(vis=True)
parser.add_argument('--gen_args', nargs='+', default=[10,1], type=float, help='Arguments to the curve generator.')
args = parser.parse_args()

curve_gen = importlib.import_module('DroneExperiments.core.refs.'+args.type)

def gen_xref_uref(xyz):
    
    xyz = partial(xyz, args.gen_args)
    xref, uref, t = gen_xref_uref_from_xyz(xyz, args.T, dt, translate = True)
    print("xref",xref.shape)
    start_position = xref[0,:3]

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # Plot the 3D curve
    ax.plot(xref[:, 0], xref[:, 1], xref[:, 2], label='3D Spiral Curve')
    # Adding labels and title
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')
    ax.set_title('3D Spiral Curve')
    # Adding a legend
    ax.legend()
    # Display the 3D plot
    plt.savefig(f"plt_{args.gen_args}.png")


    if args.save:
        with open(args.save, 'wb') as handle:
            pickle.dump({'start_position': start_position, 'xref': xref, 'uref': uref}, handle, protocol=pickle.HIGHEST_PROTOCOL)

    if args.vis:
        fig, (xy, xz, timed) = plt.subplots(3, 1, figsize=(10, 6))

        xy.plot(xref[:,0], xref[:,1])
        xy.set_xlabel('x')
        xy.set_ylabel('y')
        
        xz.plot(xref[:,0], xref[:,2])
        xz.set_xlabel('x')
        xz.set_ylabel('z')
        
        timed.plot(t, xref[:, 0], label='x')
        timed.plot(t, xref[:, 1], label='y')
        timed.plot(t, xref[:, 2], label='z')
        timed.plot(t, xref[:, 3], label='vx')
        timed.plot(t, xref[:, 4], label='vy')
        timed.plot(t, xref[:, 5], label='vz')
        timed.plot(t, xref[:, 6], label='ax')
        timed.plot(t, xref[:, 7], label='ay')
        timed.plot(t, xref[:, 8], label='az')
        timed.legend()

        plt.savefig(f"ref_{args.gen_args}.png")

gen_xref_uref(curve_gen.xyz)    
print("Done")