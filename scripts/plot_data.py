from matplotlib import pyplot as plt
import numpy as np
import sys
import pickle5 as pickle

def gaussian(x, s):
    return 1./np.sqrt(2. * np.pi * s**2) * np.exp(-x**2 / (2. * s**2))

def gaussian_filter(x, w=1, s=1.):
    padded_x = np.array([x[0], ] * w + x.tolist() + [x[-1], ] * w)
    kernel = np.fromiter((gaussian(_x, 1) for _x in range(-w, w+1, 1)), np.float32)
    return np.convolve(padded_x, kernel, mode='valid')

def moving_average(x, decay = 0.7):
    out = []
    _f = 0.
    for _x in x:
        _f = _f * decay + (1-decay) * _x
        out.append(_f)
    return np.array(out)

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
    v = states[:, 3 + idx]
    a = states[:, 6 + idx]
    u = controls[:, 0 + idx]

    # smooth_v = gaussian_filter(v, 1)

    # plt.plot(t, x - x.mean(), label='pos')
    # plt.plot(t, x_fused - x_fused.mean(), label='pos_fused')
    # plt.plot(t, v, label='vel')
    # plt.plot(t, v_fused, label='vel_fused')
    # plt.plot(t[:-1], (v_fused[1:] - v_fused[:-1]) / dt, label='accel_gt')
    # plt.plot(t, np.cumsum(v_fused) * dt + x[0] - x.mean(), label='pos-int')
    # plt.plot(t, smooth_x, label='pos-smooth')
    # plt.plot(t, smooth_v, label='vel-der')
    # plt.plot(t, x - x.mean(), label='pos')
    plt.plot(t, x, label='pos')
    plt.plot(t, v, label='vel')
    # plt.plot(t[:-1], (x[1:] - x[:-1]) / dt, label='vel_gt')
    plt.plot(t, a, label='acc')
    # plt.plot(t[:-1], (v[1:] - v[:-1]) / dt, label='acc_gt')
    # plt.plot(t[:-1], moving_average((v[1:] - v[:-1]) / dt), 'o-', label='accel')
    # plt.plot(t, smooth_v, 'o-', label='vel-smoothed')
    # plt.plot(t[:-1], (smooth_v[1:] - smooth_v[:-1]) / dt, 'o-', label='acc_gt')
    # plt.plot(t[:-1], (v[1:] - v[:-1]) / dt, label='accel_gt')

    plt.plot(t, u, label='cmd')
    plt.legend()
    plt.show()
