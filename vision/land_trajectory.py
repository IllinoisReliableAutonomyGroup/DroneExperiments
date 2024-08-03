import numpy as np
import matplotlib.pyplot as plt

# Take picutre of a cielcle arodn marker at fixed radius and angle
# caclulate rtrajactru
# plto on the picture using projection
def xyz(period, drone_pos,drone_angles, land_pos, sizex=1., sizey=3.):

    yaw = drone_angles[2]

    # Construct new plane by 2 points and vertical angle 
    # projected drone pos coordinate:
    drone_x_proj = np.sqrt(drone_pos[0]**2 + drone_pos[1]**2)
    drone_y_proj = drone_pos[2]
    # projected land pos coordinates:
    land_x_proj = land_pos[0]
    land_y_proj = land_pos[1]

    # Quadratic vertex form with drone pos as vertex:
    a = (land_y_proj-drone_y_proj) / (land_x_proj-drone_x_proj)**2
    def quadratic(x):
        return a*(x-drone_x_proj)**2 + drone_y_proj
    
    def rotation(theta, X):
        A = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta),  np.cos(theta), 0],
                      [0, 0, 1]])
        return A@X
    
    x = np.arange(0,drone_x_proj, drone_x_proj/period)
    y = np.zeros(len(x))
    z = quadratic(x)

    plt.plot(x, z, label='parametric curve')
    plt.xlabel('x')
    plt.ylabel('z height')
    plt.show()

    for i in range(len(x)):
        world = rotation(yaw, np.array([x[i], y[i], z[i]]))
        x[i] = world[0]
        y[i] = world[1]
        z[i] = world[2]

    return x, y, z

if __name__ == "__main__":
    period = 10
    drone_pos = [-3, -1068, 822]
    drone_angles = [0,0,0]
    land_pos = [0,0,0]
    x,y,z = xyz(period,drone_pos,drone_angles,land_pos)

    print("x",x)
    print('y',y)
    print('z',z)

    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(x, y, z, label='parametric curve')
    ax.legend()
    plt.show()

