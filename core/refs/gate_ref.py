import numpy as np

def normalize(x):
    return x / np.sqrt((x**2).sum())

def direction(x, y):
    return normalize(np.array(y) - np.array(x))

def distance(x, y):
    e = np.array(x) - np.array(y)
    return np.sqrt((e**2).sum())

def xyz(args, t, get_T=False):
    init, gate, v, before, after = args

    g_center = gate[:3]
    g_normal = gate[3:]

    p_before = g_center + before * g_normal
    p_after = g_center - after * g_normal

    T1 = distance(init, p_before) / v
    T2 = distance(p_before, p_after) / v

    if get_T:
        return T1 + T2 + 1.

    path = np.zeros([len(t), 3])

    idx1 = np.where(t > T1)[0][0]
    path[:idx1, :] = init + direction(init, p_before) * v * t[:idx1].reshape(-1, 1)

    idx3 = np.where(t > (T1 + T2))[0][0]
    path[idx1:idx3] = p_before + direction(p_before, p_after) * v * (t[idx1:idx3] - t[idx1]).reshape(-1, 1)

    path[idx3:] = p_after
    return path[:, 0], path[:, 1], path[:, 2]
