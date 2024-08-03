import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import pickle
import argparse

parser = argparse.ArgumentParser(description="")
parser.add_argument('--save', type=int, help='trajectory number to save the data.')
args = parser.parse_args()

if args.save is None:
    args.save = 0

def traj_generator(initial_point, vel, total_time, change_frequency,  traj_bound):
  
    partitions = int(total_time/change_frequency)

    waypoints = [initial_point]
    for p in range(partitions):
        while True:

            # Determine direction
            theta = np.random.uniform(0, 2 * np.pi)
            z_direction = np.cos(theta)
            y_direction = np.sin(theta)

            displacement_vector = np.array([0, y_direction,  z_direction]) * vel *  change_frequency
            next_waypoint = waypoints[-1] + displacement_vector

            # Check if point is within safety flight bounds
            if(next_waypoint[0] <traj_bound[0] or next_waypoint[0] > traj_bound[1] or
            next_waypoint[1] <traj_bound[2] or next_waypoint[1] > traj_bound[3] or
            next_waypoint[2] <traj_bound[4] or next_waypoint[2] > traj_bound[5]):
                print(f"re-computing points {p} out of {partitions}...")
                continue
            else:
                break
        
        print("adding new waypoint")
        waypoints.append(next_waypoint)

    waypoints = np.array(waypoints)
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    z = waypoints[:, 2]

    # Fit cubic splines to each dimension
    cs_x = CubicSpline(np.arange(len(x)), x)
    cs_y = CubicSpline(np.arange(len(y)), y)
    cs_z = CubicSpline(np.arange(len(z)), z)

    # Save cubic splines to a file
    with open(f'traj_cubic_splines_{args.save}.pkl', 'wb') as f:
        pickle.dump({'cs_x': cs_x, 'cs_y': cs_y, 'cs_z': cs_z}, f)

    return x,y,z

def main(total_time, dt, waypoint_x, waypoint_y, waypoint_z):

    # Load cubic splines from a file
    with open(f'traj_cubic_splines_{args.save}.pkl', 'rb') as f:
        splines = pickle.load(f)
    cs_x = splines['cs_x']
    cs_y = splines['cs_y']
    cs_z = splines['cs_z']


    # Define a range for interpolation
    t_fine = np.linspace(0, len(cs_x.x) - 1, int(total_time/dt))

    # Interpolate the spline fits
    x_smooth = cs_x(t_fine)
    y_smooth = cs_y(t_fine)
    z_smooth = cs_z(t_fine)

    # Combine smooth points into a single array
    smooth_points = np.vstack((x_smooth, y_smooth, z_smooth)).T

    # Plotting
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot waypoints
    ax.plot(waypoint_x, waypoint_y, waypoint_z, 'ro', label='Waypoints')

    # Plot smooth path
    ax.plot(x_smooth, y_smooth, z_smooth, '-',color='blue', linewidth = 0.5, label='Smooth Path')

    y_limits = ax.get_ylim()
    z_limits = ax.get_zlim()
    min_limit = min(y_limits[0], z_limits[0])
    max_limit = max(y_limits[1], z_limits[1])
    ax.set_ylim([min_limit, max_limit])
    ax.set_zlim([0, max_limit])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # ax.set_box_aspect([1,1,1])
    ax.legend()
    plt.title('UAV Smooth Trajectory')
    plt.show()





if __name__ == "__main__":

    ############################## USER INPUTS ##############################
    vel = 0.5
    total_time = 30 #s
    change_frequency = 5.0 #s
    dt = 0.05 #s

    start_waypoint = np.array([0,0,1])

    # traj_bound = [ min/max X, min/max Y, min/max Z ]
    traj_bound = [-0.5, 0.3, -2.5, 3.4, 0.5, 2.5]

    #########################################################################
    waypoint_x, waypoint_y, waypoint_z = traj_generator(start_waypoint, vel, total_time, change_frequency,  traj_bound)

    downsample_mult = 50
    dt = dt / downsample_mult
    t = np.arange(0., total_time, dt)

    main(total_time, dt, waypoint_x, waypoint_y, waypoint_z)



