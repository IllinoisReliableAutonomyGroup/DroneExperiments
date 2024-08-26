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
    x = a * np.sin(t)
    z = b * np.cos(t)

    y = np.ones_like(x) * (-3.7) # 1
    x += 0.5
    z += 1.5

    return x, y, z
