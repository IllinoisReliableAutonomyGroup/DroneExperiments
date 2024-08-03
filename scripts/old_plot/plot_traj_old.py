from matplotlib import pyplot as plt
import numpy as np
import argparse
import sys
import pickle5 as pickle

parser = argparse.ArgumentParser(description="")
parser.add_argument('--ref', type=str, help='filename to the reference trajectories.')
parser.add_argument('--traj', type=str, help='filename to the actual trajectories.')
args = parser.parse_args()

with open(args.traj, 'rb') as handle:
    data = pickle.load(handle)

dt = data[1]
T = data[2]
real_t = np.arange(0, T, dt)

actual_traj = np.array(data[0][0][0])
import ipdb; ipdb.set_trace()
u = np.array(data[0][0][1])

with open(args.ref, 'rb') as handle:
    ref = pickle.load(handle)
ref_traj = ref['xref']

x = actual_traj[:, 0]
y = actual_traj[:, 1]
z = actual_traj[:, 2]
vx = actual_traj[:, 3]
vy = actual_traj[:, 4]
vz = actual_traj[:, 5]
ax = actual_traj[:, 6]
ay = actual_traj[:, 7]
az = actual_traj[:, 8]

ref_x = ref_traj[:, 0]
ref_y = ref_traj[:, 1]
ref_z = ref_traj[:, 2]
ref_vx = ref_traj[:, 3]
ref_vy = ref_traj[:, 4]
ref_vz = ref_traj[:, 5]
ref_ax = ref_traj[:, 6]
ref_ay = ref_traj[:, 7]
ref_az = ref_traj[:, 8]

fig = plt.figure(figsize=(20, 10))
xy = fig.add_subplot(2, 1, 1)
timed = fig.add_subplot(2, 1, 2)

xy.plot(ref_x, ref_y, 'k-', label='ref')
xy.plot(x, y, 'g-', label='actual')
xy.set_xlabel('x')
xy.set_ylabel('y')
xy.legend()

# timed.plot(real_t, x, label='x')
# timed.plot(real_t, y, label='y')
# timed.plot(real_t, z, label='z')
timed.plot(real_t, vx, label='vx')
timed.plot(real_t, vy, label='vy')
#timed.plot(real_t, vz, label='vz')
timed.plot(real_t, ax, label='ax')
timed.plot(real_t, ay, label='ay')
#timed.plot(real_t, az, label='az')
# timed.plot(real_t, ref_x, label='ref_x')
# timed.plot(real_t, ref_y, label='ref_y')
# timed.plot(real_t, ref_z, label='ref_z')
# timed.plot(real_t, ref_vx, label='ref_vx')
# timed.plot(real_t, ref_vy, label='ref_vy')
# timed.plot(real_t, ref_vz, label='ref_vz')
#timed.plot(real_t, ref_ax, label='ref_ax')
#timed.plot(real_t, ref_ay, label='ref_ay')
#timed.plot(real_t, ref_az, label='ref_az')
#timed.plot(real_t, u[:, 0], label='cmd_ax')
#timed.plot(real_t, u[:, 1], label='cmd_ay')
#timed.plot(real_t, u[:, 2], label='cmd_az')
timed.legend()

plt.show()
