#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
import argparse
import time
import pickle5 as pickle
from DroneExperiments.core.constants import dt
from DroneExperiments.ros.DroneInterface import Drone

parser = argparse.ArgumentParser(description="")
parser.add_argument('--ref', type=str, help='filename to the reference trajectories.')
args = parser.parse_args()

class Visualization2D():
    def __init__(self, ref=None):
        self.actual_traj = []
        plt.ion()
        self.fig = plt.figure()
        self.ax = plt.axes()
        self.obj_actual_current = self.ax.plot([], [], 'go', markersize=5.)[0]
        self.obj_actual_traj = self.ax.plot([], [], 'g-')[0]
        # self.obj_ref_current = self.ax.plot([], [], 'ko', markersize=5.)
        if ref:
            with open(ref, 'rb') as handle:
                ref = pickle.load(handle)
            self.ref_traj = ref['xref'][:, 0:2]
            self.obj_ref_traj = self.ax.plot(self.ref_traj[:, 0], self.ref_traj[:, 1], 'k-')[0]
            margin = 1.
            self.ax.set_xlim(xmin=ref['xref'][:, 0].min() - margin, xmax=ref['xref'][:, 0].max() + margin)
            self.ax.set_ylim(ymin=ref['xref'][:, 1].min() - margin, ymax=ref['xref'][:, 1].max() + margin)
        else:
            self.ref_traj = None
            self.obj_ref_traj = None

    def update(self, state, current_cmd=None):
        self.ax.set_title('t = %.3f s'%rospy.get_time())
        self.actual_traj.append(state[:3].copy())
        self.obj_actual_current.set_xdata(state[0])
        self.obj_actual_current.set_ydata(state[1])
        atraj = np.array(self.actual_traj)
        # print(atraj.shape)
        self.obj_actual_traj.set_xdata(atraj[:, 0])
        self.obj_actual_traj.set_ydata(atraj[:, 1])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main():
    rospy.init_node('visualization2D', anonymous=True)
    drone = Drone()
    vis = Visualization2D(args.ref)

    rate = rospy.Rate(1. / dt)

    state = drone.state

    while not rospy.is_shutdown():
        vis.update(state)
        rate.sleep()

if __name__ == "__main__":
    main()
