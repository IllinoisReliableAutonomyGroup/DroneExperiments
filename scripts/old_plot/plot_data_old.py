from matplotlib import pyplot as plt
import numpy as np
import sys
import pickle5 as pickle

def moving_average(x, w):
    x = np.array([x[-1], ] * int((w - 1) / 2) + x.tolist() + [x[-1], ] * int((w - 1) / 2))
    return np.convolve(x, np.ones(w), 'valid') / w

with open(sys.argv[1], 'rb') as handle:
    data = pickle.load(handle)

# from IPython import embed; embed()

dt = data[1]
T = data[2]
trajs = data[0]

idx = 0

for idx_traj in range(len(trajs)):
    states = np.array(trajs[idx_traj][0])
    controls = np.array(trajs[idx_traj][1])
    t = np.array(list(range(states.shape[0]))) * dt
    x = states[:, 0 + idx]
    v = states[:, 7 + idx]
    x_fused = states[:, 10 + idx]
    v_fused = states[:, 17 + idx]
    u = controls[:, 0 + idx]

    # for plotting
    # x = x_fused
    v = v_fused

    # smooth_x = moving_average(x, 11)
    # smooth_v = (smooth_x[1:] - smooth_x[:-1]) / dt
    # smooth_v = np.array(smooth_v.tolist() + [0.])
    smooth_v = moving_average(v, 11)

    # plt.plot(t, x - x.mean(), label='pos')
    # plt.plot(t, x_fused - x_fused.mean(), label='pos_fused')
    # plt.plot(t, v, label='vel')
    # plt.plot(t, v_fused, label='vel_fused')
    # plt.plot(t[:-1], (v_fused[1:] - v_fused[:-1]) / dt, label='accel_gt')
    # plt.plot(t, np.cumsum(v_fused) * dt + x[0] - x.mean(), label='pos-int')
    # plt.plot(t, smooth_x, label='pos-smooth')
    # plt.plot(t, smooth_v, label='vel-der')
    plt.plot(t, x - x.mean(), label='pos')
    # plt.plot(t, v, label='vel')
    plt.plot(t, smooth_v, label='vel-smoothed')
    plt.plot(t[:-1], (smooth_v[1:] - smooth_v[:-1]) / dt, label='accel_gt')

    plt.plot(t, u, label='cmd')
    plt.legend()
    plt.show()
