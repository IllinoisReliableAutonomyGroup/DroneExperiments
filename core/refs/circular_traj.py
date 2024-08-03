import numpy as np

def xyz(args, real_t):
    period, size_radius, phi = args
    print('period/size radius/phi: ', period, size_radius, phi)

    # Phi angle introduces circle tilt in 3D along y-axis

    centerx = 0
    centery = 0
    centerz = 2

    t = (real_t / period) * (2 * np.pi)

    x = centerx + size_radius * np.cos(t) * np.cos(phi)
    y = centery + size_radius * np.sin(t) 
    z = centerz + size_radius * np.cos(t) * np.sin(phi)
    
    return x, y, z
