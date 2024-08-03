import numpy as np

def normalize(x):
    return x / np.sqrt((x**2).sum())

def direction(x, y):
    return normalize(np.array(y) - np.array(x))

def distance(x, y):
    e = np.array(x) - np.array(y)
    return np.sqrt((e**2).sum())

def xyz(args, t):
    init, goal, v_approach, v_landing, time_hover = args
    x0, y0, z0 = init
    xf, yf, zf = goal

    # XY
    T1 = distance([x0, y0], [xf, yf]) / v_approach
    idx1 = np.where(t > T1)[0][0]
    path = np.zeros([len(t), 3])
    path[:idx1, :] = np.array(init) + direction(init, [xf, yf, z0]) * v_approach * t[:idx1].reshape(-1, 1)

    # hover
    idx2 = np.where(t > T1 + time_hover)[0][0]
    path[idx1:idx2, :] = [xf, yf, z0]

    # landing
    T2 = distance([z0,], [zf,]) / v_landing
    idx3 = np.where(t > (T1 + time_hover + T2))[0][0]
    path[idx2:idx3, :] = np.array([xf, yf, z0]) + direction([xf, yf, z0], goal) * v_landing * (t[idx2:idx3] - t[idx2]).reshape(-1, 1)

    # other
    path[idx3:] = np.array(goal)
    return path[:, 0], path[:, 1], path[:, 2]
