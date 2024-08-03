import numpy as np
def xyz(t):
    x = np.zeros_like(t)
    y = np.zeros_like(t)
    z = np.ones_like(x) * 1.5
    return x, y, z
