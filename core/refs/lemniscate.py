import numpy as np

def xyz(args, real_t):
    period, sizex, sizey = args
    print('period/sizex/sizey: ', period, sizex, sizey)
    if not period:
        period = real_t[-1]
    t = real_t / period * 2 * np.pi
    x = np.sqrt(2) * np.cos(t) / (1 + np.sin(t) ** 2)
    y = x * np.sin(t)
    x = sizex * x
    y = sizey * y
    z = np.ones_like(x) * 1.5
    return x, y, z

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D

    x,y,z = xyz([10,1,1],0.01 )

    # Create a figure and 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot a 3D scatter plot
    ax.scatter(x, y, z, c='r', marker='o')

    # Set labels for the axes
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # Set a title for the plot
    ax.set_title('3D Scatter Plot')

    # Show the plot
    plt.show()
