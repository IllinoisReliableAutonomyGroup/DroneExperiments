import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.signal import savgol_filter


df = pd.read_csv('leader_nomarker_ellip_5_2_05.csv')

# Convert DataFrame to numpy array
data_array = df.values

column_headings = df.columns.tolist()

print("Column Headings:")
print(column_headings)

x_c = df['TX'].tolist()
y_c = df['TY'].tolist()
z_c = df['TZ'].tolist()

x_c = np.array(x_c)/1000
y_c = np.array(y_c)/1000
z_c = np.array(z_c)/1000

times = np.arange(0,len(x_c))*0.01

print(f"size of data: ", x_c.shape)

#lower and upper bounds
lower = 1250
upper = 5500

x_c=x_c[lower:upper]
y_c=y_c[lower:upper]
z_c=z_c[lower:upper]

# Number of data points
n = x_c[:].shape[0]
dt = 0.01

# Initialize velocity arrays
v_x = np.zeros(n)
v_y = np.zeros(n)
v_z = np.zeros(n)



#Initialize accleration arrays
a_x = np.zeros(n)
a_y = np.zeros(n)
a_z = np.zeros(n)

#11 point central difference for velocity
for i in range(5, n-5):
    # v_x[i] = (-2*x_c[i-5]+25*x_c[i-4]-150*x_c[i-3]+600*x_c[i-2]-2100*(x_c[i-1])+2100*(x_c[i+1])-600*x_c[i+2]+150*x_c[i+3]-25*x_c[i+4]+2*x_c[i+5]) / (2520 * dt)
    # v_y[i] = (-2*y_c[i-5]+25*y_c[i-4]-150*y_c[i-3]+600*y_c[i-2]-2100*(y_c[i-1])+2100*(y_c[i+1])-600*y_c[i+2]+150*y_c[i+3]-25*y_c[i+4]+2*y_c[i+5]) / (2520 * dt)
    # v_z[i] = (-2*z_c[i-5]+25*z_c[i-4]-150*z_c[i-3]+600*z_c[i-2]-2100*(z_c[i-1])+2100*(z_c[i+1])-600*z_c[i+2]+150*z_c[i+3]-25*z_c[i+4]+2*z_c[i+5]) / (2520 * dt)

    v_x[i] = (x_c[i-2]-8*(x_c[i-1])+8*(x_c[i+1])-x_c[i+2]) / (12 * dt)
    v_y[i] = (y_c[i-2]-8*(y_c[i-1])+8*(y_c[i+1])-y_c[i+2]) / (12 * dt)
    v_z[i] = (z_c[i-2]-8*(z_c[i-1])+8*(z_c[i+1])-z_c[i+2]) / (12 * dt)

for i in range(3, n-3):
    # a_x[i] = (-2*v_x[i-5]+25*v_x[i-4]-150*v_x[i-3]+600*v_x[i-2]-2100*(v_x[i-1])+2100*(v_x[i+1])-600*v_x[i+2]+150*v_x[i+3]-25*v_x[i+4]+2*v_x[i+5]) / (2520 * dt)
    # a_y[i] = (-2*v_y[i-5]+25*v_y[i-4]-150*v_y[i-3]+600*v_y[i-2]-2100*(v_y[i-1])+2100*(v_y[i+1])-600*v_y[i+2]+150*v_y[i+3]-25*v_y[i+4]+2*v_y[i+5]) / (2520 * dt)
    # a_z[i] = (-2*v_z[i-5]+25*v_z[i-4]-150*v_z[i-3]+600*v_z[i-2]-2100*(v_z[i-1])+2100*(v_z[i+1])-600*v_z[i+2]+150*v_z[i+3]-25*v_z[i+4]+2*v_z[i+5]) / (2520 * dt)
    a_x[i] = (2*x_c[i-3]-27*x_c[i-2]+270*(x_c[i-1])-490*x_c[i]+270*(x_c[i+1])-27*x_c[i+2]+2*x_c[i+3]) / (180 * dt**2)
    a_y[i] = (2*y_c[i-3]-27*y_c[i-2]+270*(y_c[i-1])-490*y_c[i]+270*(y_c[i+1])-27*y_c[i+2]+2*y_c[i+3]) / (180 * dt**2)
    a_z[i] = (2*z_c[i-3]-27*z_c[i-2]+270*(z_c[i-1])-490*z_c[i]+270*(z_c[i+1])-27*z_c[i+2]+2*z_c[i+3]) / (180 * dt**2)

print(f"x velocity max: {max(v_x)}")
print(f"y velocity max: {max(v_y)}")
print(f"z velocity max: {max(v_z)}")

print(f"x acceleration max: {max(a_x)}")
print(f"y acceleration max: {max(a_y)}")
print(f"z acceleration max: {max(a_z)}")

# Forward difference for the first data point
v_x[0] = (x_c[lower:upper][1] - x_c[lower:upper][0]) / dt
v_y[0] = (y_c[lower:upper][1] - y_c[lower:upper][0]) / dt
v_z[0] = (z_c[lower:upper][1] - z_c[lower:upper][0]) / dt

a_x[0] = (v_x[lower:upper][1] - v_x[lower:upper][0]) / dt
a_y[0] = (v_y[lower:upper][1] - v_y[lower:upper][0]) / dt
a_z[0] = (v_z[lower:upper][1] - v_z[lower:upper][0]) / dt

# Backward difference for the last data point
v_x[-1] = (x_c[lower:upper][-1] - x_c[lower:upper][-2]) / dt
v_y[-1] = (y_c[lower:upper][-1] - y_c[lower:upper][-2]) / dt
v_z[-1] = (z_c[lower:upper][-1] - z_c[lower:upper][-2]) / dt

a_x[-1] = (v_x[lower:upper][-1] - v_x[lower:upper][-2]) / dt
a_y[-1] = (v_y[lower:upper][-1] - v_y[lower:upper][-2]) / dt
a_z[-1] = (v_z[lower:upper][-1] - v_z[lower:upper][-2]) / dt


#finds which plots the user wants to see
gettingUserInput = True
plotsToShow = []
while gettingUserInput:
    plotsAdd = input("Which plots do you want? Type a number 1-4 (1 = '3dposition', 2 = 'position', 3 = 'velocity', 4 = 'acceleration): ")
    plotsToShow.append(plotsAdd)
    print(plotsToShow)
    continueadding = input("Add more plots?(yes, no): ")
    if continueadding == 'yes':
        continue
    elif continueadding == 'no':
        gettingUserInput = False
    else:
        print("invalid input")

#asks if user wants animations
while True:
    animationUserInput = input("Add animation? (yes, no): ")
    if animationUserInput == 'yes':
        showAnimations = True
        break
    elif animationUserInput == 'no':
        showAnimations = False
        break
    else: 
        print("invalid input")
        


if '1' in plotsToShow:
    if showAnimations:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Initialize the plot
        line, = ax.plot(x_c[lower:upper], y_c[lower:upper], z_c[lower:upper], label='Chaser Drone')

        # Function to initialize the animation
        def init():
            line.set_data([], [])
            line.set_3d_properties([])
            return line,

        # Function to update the animation at each frame
        def update(i):
            line.set_data(x_c[:i], y_c[:i])
            line.set_3d_properties(z_c[:i])
            return line,

        # Create the animation
        ani = FuncAnimation(fig, update, frames=range(lower, upper), init_func=init, blit=True, interval=0.001)


        ax.set_xlabel('X Position (m)',fontsize = 14)
        ax.set_ylabel('Y Position (m)',fontsize = 14)
        ax.set_zlabel('Z Position (m)',fontsize = 14)
        ax.legend(fontsize = 14)
        plt.show()
    else:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(x_c[lower:upper], y_c[lower:upper], z_c[lower:upper],label='Chaser Drone')

        ax.set_xlabel('X Position (m)',fontsize = 14)
        ax.set_ylabel('Y Position (m)',fontsize = 14)
        ax.set_zlabel('Z Position (m)',fontsize = 14)
        ax.legend(fontsize = 14)
        plt.show()

if '2' in plotsToShow:
    if showAnimations:
        #position
        time = times[lower:upper]-times[lower]
        fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(12, 8))

        xline, = xval.plot([], [], label='Chaser Drone')
        yline, = yval.plot([], [])
        zline, = zval.plot([], [])

        # Set plot limits
        xval.set_xlim(time[0], time[-1])
        yval.set_xlim(time[0], time[-1])
        zval.set_xlim(time[0], time[-1])

        xval.set_ylim(np.min(x_c)-0.25, np.max(x_c)+0.25)
        yval.set_ylim(np.min(y_c)-0.25, np.max(y_c)+0.25)
        zval.set_ylim(np.min(z_c)-0.25, np.max(z_c)+0.25)

        # Function to initialize the animation
        def init():
            xline.set_data([], [])
            yline.set_data([], [])
            zline.set_data([], [])
            return xline, yline, zline

        # Function to update the animation at each frame
        def update(i):
            xline.set_data(time[:i], x_c[:i])
            yline.set_data(time[:i], y_c[:i])
            zline.set_data(time[:i], z_c[:i])
            return xline, yline, zline

        # Create the animation
        ani = FuncAnimation(fig, update, frames=range(1, upper-lower), init_func=init, blit=True, interval=0.001)

        xval.legend()
        plt.tight_layout()
        plt.show()
    else:
        time = times[lower:upper]-times[lower]
        fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(12, 8))
        xval.plot(time,x_c, label = "Chaser Drone")

        # xval.legend()  
        yval.plot(time,y_c)

        # yval.legend()
        zval.plot(time,z_c)

        # zval.legend()

        plt.show()

if '3' in plotsToShow:
    if showAnimations:
        #velocity
        time = times[lower:upper]-times[lower]
        fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(12, 8))

        xline, = xval.plot([], [], label='Chaser Drone')
        yline, = yval.plot([], [])
        zline, = zval.plot([], [])

        # Set plot limits
        xval.set_xlim(time[0], time[-1])
        yval.set_xlim(time[0], time[-1])
        zval.set_xlim(time[0], time[-1])

        xval.set_ylim(np.min(v_x)-0.25, np.max(v_x)+0.25)
        yval.set_ylim(np.min(v_y)-0.25, np.max(v_y)+0.25)
        zval.set_ylim(np.min(v_z)-0.25, np.max(v_z)+0.25)

        # Function to initialize the animation
        def init():
            xline.set_data([], [])
            yline.set_data([], [])
            zline.set_data([], [])
            return xline, yline, zline

        # Function to update the animation at each frame
        def update(i):
            xline.set_data(time[:i], v_x[:i])
            yline.set_data(time[:i], v_y[:i])
            zline.set_data(time[:i], v_z[:i])
            return xline, yline, zline

        # Create the animation
        ani = FuncAnimation(fig, update, frames=range(1, upper-lower), init_func=init, blit=True, interval=0.001)

        xval.legend()
        plt.tight_layout()
        plt.show()
    else: 
        time = times[lower:upper]-times[lower]
        fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(12, 8))
        xval.plot(time,v_x, label = "Chaser Drone velocity") 
        yval.plot(time,v_y)
        zval.plot(time,v_z)
        plt.show()



# time = times[lower:upper]-times[lower]
# fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(12, 8))
# xval.plot(time,a_x[lower:upper], label = "Chaser Drone acceleration") 
# yval.plot(time,a_y[lower:upper])
# zval.plot(time,a_z[lower:upper])
# xval.set_ylim([-10, 10])
# yval.set_ylim([-10, 10])
# zval.set_ylim([-10, 10])
# plt.show()

# times = np.arange(0,len(a_x))*0.01
if '4' in plotsToShow:
    #acceleration

    # Smoothing the data using Savitzky-Golay filter
    print("Smoothing filters for acceleration. Larger window length increases smoothness but looses detail.\nSmaller polynomial order increases smoothness but looses detail.\nWindow length must an odd integer and be bigger than the polynomial order")
    window_length = int(input("Choose window length: "))  # Choose an odd number
    polyorder = int(input("Choose polynomial order: "))  # Polynomial order
    ax_smooth = savgol_filter(a_x, window_length, polyorder)
    ay_smooth = savgol_filter(a_y, window_length, polyorder)
    az_smooth = savgol_filter(a_z, window_length, polyorder)
    if showAnimations:
        time = times[lower:upper]-times[lower]
        fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(12, 8))

        xline, = xval.plot([], [], label='Chaser Drone')
        yline, = yval.plot([], [])
        zline, = zval.plot([], [])

        # Set plot limits
        xval.set_xlim(time[0], time[-1])
        yval.set_xlim(time[0], time[-1])
        zval.set_xlim(time[0], time[-1])

        xval.set_ylim([-10, 10])
        yval.set_ylim([-10, 10])
        zval.set_ylim([-10, 10])

        # Function to initialize the animation
        def init():
            xline.set_data([], [])
            yline.set_data([], [])
            zline.set_data([], [])
            return xline, yline, zline

        # Function to update the animation at each frame
        def update(i):
            xline.set_data(time[:i], ax_smooth[:i])
            yline.set_data(time[:i], ay_smooth[:i])
            zline.set_data(time[:i], az_smooth[:i])
            return xline, yline, zline

        # Create the animation
        ani = FuncAnimation(fig, update, frames=range(1, upper-lower), init_func=init, blit=True, interval=0.001)

        xval.legend()
        plt.tight_layout()
        plt.show()
    else:
        time = times[lower:upper]-times[lower]
        fig, (xval, yval, zval) = plt.subplots(3, 1, figsize=(12, 8))
        xval.plot(time,ax_smooth, label = "Chaser Drone acceleration") 
        yval.plot(time,ay_smooth)
        zval.plot(time,az_smooth)
        xval.set_ylim([-10, 10])
        yval.set_ylim([-10, 10])
        zval.set_ylim([-10, 10])
        plt.show()
