import numpy as np

# Generate Ellipsoidal trajectory in YZ Plane
# xyz in global vicon coordinate frame

def xyz(args, real_t):
    period, sizex, sizey = args
    print('period/sizex/sizey: ', period, sizex, sizey)

    if not period:
        period = real_t[-1]

    t = real_t / period * 2 * np.pi

    # Parameters for the 2D ellipsoid
    a = sizex  # semi-major axis in x direction
    b = sizey  # semi-major axis in y direction
    
    # Parametric equations for a 2D ellipse
    y = a * np.sin(t)
    z = b * np.cos(t)

    x = np.ones_like(y) * 1#3.22
    z += 1.5

    return x, y, z
