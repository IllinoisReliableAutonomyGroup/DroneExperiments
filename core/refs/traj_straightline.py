import numpy as np

def xyz(args, timestep):
    period, amplitude, cycles = args
 
    t = (timestep/period) * (2*np.pi)

    x = np.sin(t)*amplitude

    y = np.ones_like(x) * -2.6
    z = np.ones_like(x) * 1.2

    return x, y, z

import numpy as np
