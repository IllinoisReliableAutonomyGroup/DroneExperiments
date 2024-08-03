import numpy as np
np.random.seed(1024)
from DroneExperiments.core.constants import dt

dt_int = 0.001
downsample_mult = int(round(dt / dt_int))
safety_area = [[-3.5, 3.], [-2., 3.5], [0.5, 2.5]]
safety_area_stop = [
    [safety_area[0][0]+0.5, safety_area[0][1]-0.5],
    [safety_area[1][0]+0.5, safety_area[1][1]-0.5],
    [safety_area[2][0]+0.5, safety_area[2][1]-0.5]]
time_wait_for_starting = 5.
time_wait_for_stoping = 3.

def get_v_and_x(accel):
    v = np.cumsum(accel) * dt_int
    v[1:] = v[0:-1]
    v[0] = 0.
    x = np.cumsum(v) * dt_int
    return v, x

def translate_x(x, idx):
    xmin, xmax = np.min(x), np.max(x)
    if xmax - xmin > safety_area_stop[idx][1] - safety_area_stop[idx][0]:
        return None
    else:
        return (safety_area_stop[idx][0] + safety_area_stop[idx][1]) / 2 - (xmin + xmax) / 2 + x

def get_start_position(signal, idx):
    v, x = get_v_and_x(signal)
    x = translate_x(x, idx)
    return x[0] if x is not None else None, v, x

def freq2temporal(t, freqs, freqs_mag_sin, freqs_mag_cos, max_mag = 1.):
    signal = 0.
    for a_s, a_c, f in zip(freqs_mag_sin, freqs_mag_cos, freqs):
        signal += a_s * np.sin(2 * np.pi * f * t) + a_c * np.cos(2 * np.pi * f * t)
    if max_mag and np.abs(signal).max() > max_mag:
        signal = signal / np.abs(signal).max() * max_mag
    return signal

def gen_control_signal(T=3., xyz=[True, False, False], max_mags=[3.,3.,3.], vis=False):
    t = np.arange(0., T, dt_int)
    control_signal = []
    start_position = []
    vs = []
    xs = []
    for idx in range(3):
        while True:
            if xyz[idx]:
                # freqs = np.linspace(0., 5, 9)
                # multipliers = np.logspace(np.log(1) / np.log(10), np.log(0.001) / np.log(10), len(freqs))
                # # multipliers = np.linspace(1., 0.1, len(freqs))
                # # multipliers = 1.
                # freqs_mag_sin = (np.random.rand(len(freqs)) - 0.5) * 2 * multipliers
                # freqs_mag_cos = (np.random.rand(len(freqs)) - 0.5) * 2 * multipliers
                freqs = [1 / 3., ]
                freqs_mag_sin = [0,]
                freqs_mag_cos = [2.,]
            else:
                freqs = [0,]
                freqs_mag_sin = [0,]
                freqs_mag_cos = [0,]
            signal = freq2temporal(t, freqs, freqs_mag_sin, freqs_mag_cos, max_mags[idx])
            x0, v, x = get_start_position(signal, idx)
            if x0 is not None:
                break
        control_signal.append(signal[::downsample_mult])
        start_position.append(x0)
        vs.append(v)
        xs.append(x)
    control_signal = np.vstack(control_signal).T
    if vis:
        from matplotlib import pyplot as plt
        plt.plot(t, vs[0], label='vx')
        plt.plot(t, xs[0], label='x')
        plt.plot(t, vs[1], label='vy')
        plt.plot(t, xs[1], label='y')
        plt.plot(t, vs[2], label='vz')
        plt.plot(t, xs[2], label='z')
        plt.legend()
        plt.show()
    return control_signal, start_position

def gen_control_signal_pulse(T=3., pulse_width=1., pulse_height=1., xyz=[True, False, False], vis=False):
    t = np.arange(0., T, dt_int)
    control_signal = []
    start_position = [0, 0, 0]
    for idx in range(3):
        signal = np.zeros_like(t)
        if xyz[idx]:
            prefix = 0.
            start_tidx = int(prefix / dt_int)
            end_tidx = int((prefix + pulse_width) / dt_int)
            signal[start_tidx:end_tidx] = pulse_height
        control_signal.append(signal[::downsample_mult])
    control_signal = np.vstack(control_signal).T
    return control_signal, start_position
