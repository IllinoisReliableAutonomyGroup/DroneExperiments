import numpy as np
from DroneExperiments.sysid.ExcitationGenerator import translate_x
from DroneExperiments.core.constants import x_Tc, y_Tc, z_Tc
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def gen_xref_uref_from_xyz(xyz, T, dt, translate=True, debug=False ):
    downsample_mult = 50
    dt = dt / downsample_mult
    if T is None:
        T = xyz(t=None, get_T=True)
    t = np.arange(0., T, dt)

    x, y, z = xyz(t)

    if translate:
        x = translate_x(x, 0)
        y = translate_x(y, 1)
        z = translate_x(z, 2)
 
        if x is None or y is None or z is None:
            print("ERROR: translate error, xyz safety bounds exceeded")
    
    vx = (x[1:] - x[:-1]) / dt
    vy = (y[1:] - y[:-1]) / dt
    vz = (z[1:] - z[:-1]) / dt
    ax = (vx[1:] - vx[:-1]) / dt
    ay = (vy[1:] - vy[:-1]) / dt
    az = (vz[1:] - vz[:-1]) / dt
    jx = (ax[1:] - ax[:-1]) / dt
    jy = (ay[1:] - ay[:-1]) / dt
    jz = (az[1:] - az[:-1]) / dt

    t = t[:-3]
    x = x[:-3]
    y = y[:-3]
    z = z[:-3]
    vx = vx[:-2]
    vy = vy[:-2]
    vz = vz[:-2]
    ax = ax[:-1]
    ay = ay[:-1]
    az = az[:-1]

    xref = np.zeros([t.shape[0], 9])
    xref[:, 0] = x
    xref[:, 1] = y
    xref[:, 2] = z
    xref[:, 3] = vx
    xref[:, 4] = vy
    xref[:, 5] = vz
    xref[:, 6] = ax
    xref[:, 7] = ay
    xref[:, 8] = az

    # Acceleration as a linear system: a = jt + a0
    uref = np.zeros([t.shape[0], 3])
    uref[:, 0] = jx * x_Tc + ax
    uref[:, 1] = jy * y_Tc + ay
    uref[:, 2] = jz * z_Tc + az

    xref = xref[::downsample_mult]
    uref = uref[::downsample_mult]
    t = t[::downsample_mult]

    if debug:

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        # Plot a 3D scatter plot
        ax.scatter(x, y, z, c='r', marker='o')
        # Set labels for the axes
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        ax.set_title('3D Scatter Plot')
        # plt.show()
        plt.savefig('generated_traj_3d.png')

        fig, (xaxis,yaxis,zaxis, ux, uy, uz) = plt.subplots(6, 1, figsize=(12, 6))
        xaxis.plot(t, xref[:,0], 'k-', label='ref')
        xaxis.set_xlabel('t')
        xaxis.set_ylabel('x')

        yaxis.plot(t, xref[:,1], 'k-', label='ref')
        yaxis.set_xlabel('t')
        yaxis.set_ylabel('y')

        zaxis.plot(t, xref[:,2], 'k-', label='ref')
        zaxis.set_xlabel('t')
        zaxis.set_ylabel('z')
   
        ux.plot(t, uref[:,0], 'k-', label='ref')
        ux.set_xlabel('t')
        ux.set_ylabel('ctrl u_x')

        uy.plot(t, uref[:,1], 'k-', label='ref')
        uy.set_xlabel('t')
        uy.set_ylabel('ctrl u_y')

        uz.plot(t, uref[:,2], 'k-', label='ref')
        uz.set_xlabel('t')
        uz.set_ylabel('ctrl u_z')

        plt.tight_layout()
        # plt.show()
        plt.savefig('generated_traj_ctrl.png')

    return xref, uref, t
