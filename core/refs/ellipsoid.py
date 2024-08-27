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

    x = np.ones_like(y) * 3.22 # 1
    y += 0.5
    z += 1.5

    return x, y, z

if __name__ == "__main__":
    real_t = np.arange(0,45,0.05)
    traj = xyz((15,2,0.5), real_t)
    traj = np.array(traj)
    vel = np.linalg.norm(traj[:,1:]-traj[:,:-1], axis=0)/0.05
    print(vel.max())
