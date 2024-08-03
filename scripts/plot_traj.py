from matplotlib import pyplot as plt
import numpy as np
import argparse
import sys
import pickle5 as pickle

parser = argparse.ArgumentParser(description="")
parser.add_argument('--traj', type=str, help='filename to the actual trajectories.')
parser.add_argument('--save', type=bool, default=False, help='boolean to save figure')
args = parser.parse_args()

with open(args.traj, 'rb') as handle:
    data = pickle.load(handle)

data, dt, refs = data
xs, u, ref_ids, xstars, ustars = data[0]

real_t = np.arange(0, len(xs) * dt, dt)

starting_param = 80

real_t = real_t[starting_param:]
x = xs[starting_param:, 0]
y = xs[starting_param:, 1]
z = xs[starting_param:, 2]
vx = xs[starting_param:, 3]
vy = xs[starting_param:, 4]
vz = xs[starting_param:, 5]
ax = xs[starting_param:, 6]
ay = xs[starting_param:, 7]
az = xs[starting_param:, 8]

ref_x = xstars[starting_param:, 0]
ref_y = xstars[starting_param:, 1]
ref_z = xstars[starting_param:, 2]
ref_vx = xstars[starting_param:, 3]
ref_vy = xstars[starting_param:, 4]
ref_vz = xstars[starting_param:, 5]
ref_ax = xstars[starting_param:, 6]
ref_ay = xstars[starting_param:, 7]
ref_az = xstars[starting_param:, 8]


import matplotlib.pyplot as plt  # Add this import statement

fig, (x_val, y_val, z_val) = plt.subplots(3, 1, figsize=(12, 6))

x_val.plot(ref_x, 'k-', label='ref')
x_val.set_xlabel('t')
x_val.set_ylabel('ref x')
x_val.legend()

y_val.plot(ref_y, label='ref_y')
y_val.set_xlabel('t')
y_val.set_ylabel('ref y')
y_val.legend()

z_val.plot(ref_z, label='vy')
z_val.set_xlabel('t')
z_val.set_ylabel('ref z')
z_val.legend()

# fig.suptitle(f'{args.traj}, target V: {np.round(target_velocity,2)}m/s', fontsize=16)
plt.tight_layout()
plt.savefig('debug_ellip_xref.png')



fig, (x_val, y_val, z_val) = plt.subplots(3, 1, figsize=(12, 6))

x_val.plot(ref_ax, 'k-', label='ref_ax')
x_val.set_xlabel('t')
x_val.set_ylabel('ref x')
x_val.legend()

y_val.plot(ref_ay, label='ref_ay')
y_val.set_xlabel('t')
y_val.set_ylabel('ref y')
y_val.legend()

z_val.plot(ref_az, label='ref_az')
z_val.set_xlabel('t')
z_val.set_ylabel('ref z')
z_val.legend()

# fig.suptitle(f'{args.traj}, target V: {np.round(target_velocity,2)}m/s', fontsize=16)
plt.tight_layout()
plt.savefig('debug_ellip_acel_ref.png')


    

# # Plotting the data
# fig, (acelx, acely) = plt.subplots(2, 1, figsize=(12, 6))
# # plt.plot(real_t, vx, label='vx')
# # plt.plot(real_t, ref_vx, label='ref vx')
# acelx.plot(real_t, ax, label='ax')
# acelx.plot(real_t, ref_ax, label='ref ax')
# acelx.set_xlabel('t')
# acelx.set_ylabel('ax')
# acelx.set_ylim(-5,5)
# acelx.legend()

# acely.plot(real_t, ay, label='ay')
# acely.plot(real_t, ref_ay, label='ref ay')
# acely.set_xlabel('t')
# acely.set_ylabel('ay')
# acely.set_ylim(-5,5)
# acely.legend()

# # vy = xs[:, 4]
# # vz = xs[:, 5], label='Linear Function')

# # Adding labels and a legend
# plt.title('2D Plot using Matplotlib')



# # Display the plot
# plt.show()
# # if args.save:
# #     plt.savefig("traj"+args.traj+".png")

print("Done")