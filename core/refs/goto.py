import numpy as np

def normalize(x):
    return x / np.sqrt((x**2).sum())

def direction(x, y):
    return normalize(np.array(y) - np.array(x))

def distance(x, y):
    e = np.array(x) - np.array(y)
    return np.sqrt((e**2).sum())

def xyzy(args, dt):
    init, goal, v, a, hover_time = args

    init = np.array(init)
    goal = np.array(goal)
    dir = direction(init, goal)

    L = distance(init, goal)

    if v ** 2 / a > L:
        v = np.sqrt(L * a)

    t1 = int(v / a / dt)
    t2 = t1 + int((L - v ** 2 / a) / v / dt)
    t3 = t2 + int(v / a / dt)

    t = np.arange(0, t3 * dt + hover_time, dt)
    c = np.zeros_like(t)
    c[:t1] = 0.5 * a * (t[:t1] ** 2)
    if t2 > t1:
        c[t1:t2] = c[t1-1] + v * (t[t1:t2] - t[t1-1])
    c[t2:t3] = c[t2-1] + v * (t[t2:t3] - t[t2-1]) - 0.5 * a * ((t[t2:t3] - t[t2-1]) ** 2)
    c[t3:] = L

    path = init.reshape(1, -1) + dir.reshape(1, -1) * c.reshape(-1, 1)

    return t, path[:, 0], path[:, 1], path[:, 2], np.zeros_like(path[:, 0])

if __name__ == '__main__':
    t, x, y, z, path= xyzy([[0, 0, 0], [1, 1, 1], 0.5, 1., 3], 0.05 / 50)
    from matplotlib import pyplot as plt
    plt.plot(t, x, label='x')
    plt.plot(t, y, label='y')
    plt.plot(t, z, label='z')
    plt.legend()
    plt.show()