import pickle
import numpy as np
import os


file_path = 'vicon_estimates.pkl'
current_path = os.path.dirname(os.path.abspath(__file__))

# Combine paths using os.path.join
full_path = os.path.join(current_path, file_path)

with open(full_path,'rb') as file:
    data = pickle.load(file)

data = np.array(data)
print(data)

coord_x = []
coord_y = []
coord_z = []
position = []
velocity = []
acceleration = []
time = []

for i in range(data.shape[0]):
    coord_x.append(data[i][0])
    coord_y.append(data[i][1])
    coord_z.append(data[i][2])
    position.append(data[i][0:3])
    velocity.append(data[i][3:6])
    acceleration.append(data[i][6:9])
    time.append(data[i][-1])


print(time)
# xmin_max = (-2.80, 4.08)
# ymin_max = (-3.53, 3.94)
# zmin_max = (0, 2.86)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



# Create a 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# Scatter plot
ax.scatter(coord_x[:], coord_y[:], coord_z[:])
# Set labels for the axes
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
# Show the plot
plt.show()

position = np.array(position)
velocity = np.array(velocity)
acceleration = np.array(acceleration)
time = np.array(time)
time = time-time[0]



fig, (x,y,z) = plt.subplots(3, 1, figsize=(14, 10))
x.plot(time, position[:,0], label = "GT pos x")
x.legend()  
y.plot(time, position[:,1], label = "GT pos y")
y.legend()
z.plot(time, position[:,2], label = "GT pos z")
z.legend()
plt.show()

# Find indices where values exceed a value
value =17
indices = np.where(time > value)[0]
clip_indices = indices[0]
print("clipping index: ",clip_indices)


times = np.arange(0,velocity.shape[0],1) * 0.01
fig, (vx,vy,vz) = plt.subplots(3, 1, figsize=(14, 10))
vx.plot(times[:clip_indices], velocity[:clip_indices,0], label = "GT vel x")
vx.legend()  
vy.plot(times[:clip_indices], velocity[:clip_indices,1], label = "GT vel y")
vy.legend()
vz.plot(times[:clip_indices], velocity[:clip_indices,2], label = "GT vel z")
vz.legend()
plt.show()

fig, (ax,ay,az) = plt.subplots(3, 1, figsize=(14, 10))
ax.plot(times[:clip_indices], acceleration[:clip_indices,0], label = "GT acceleration x")
ax.legend()  
ay.plot(times[:clip_indices], acceleration[:clip_indices,1], label = "GT acceleration y")
ay.legend()
az.plot(times[:clip_indices], acceleration[:clip_indices,2], label = "GT acceleration z")
az.legend()

plt.show()

print("y val max: ", np.max(velocity[:clip_indices,1]))
print("y vel min: ", np.min(velocity[:clip_indices,1]))
print("y acl max: ", np.max(acceleration[:clip_indices,1]))
print("y acl min: ", np.min(acceleration[:clip_indices,1]))


bound_upper = 5000

# fig, (px,py,pz,vx,vy,vz,ax,ay,az) = plt.subplots(9, 1, figsize=(14, 10))

# px.plot(times[:bound_upper], position[:,0][:bound_upper], label = "GT position x")
# px.legend()  
# py.plot(times[:bound_upper], position[:,1][:bound_upper], label = "GT position y")
# py.legend()
# pz.plot(times[:bound_upper], position[:,2][:bound_upper], label = "GT position z")
# pz.legend()

# vx.plot(times[:bound_upper], velocity[:,0][:bound_upper], label = "GT vel x")
# vx.legend()  
# vy.plot(times[:bound_upper], velocity[:,1][:bound_upper], label = "GT vel y")
# vy.legend()
# vz.plot(times[:bound_upper], velocity[:,2][:bound_upper], label = "GT vel z")
# vz.legend()

# ax.plot(times[:bound_upper], acceleration[:,0][:bound_upper], label = "GT acceleration x")
# ax.legend()  
# ay.plot(times[:bound_upper], acceleration[:,1][:bound_upper], label = "GT acceleration y")
# ay.legend()
# az.plot(times[:bound_upper], acceleration[:,2][:bound_upper], label = "GT acceleration z")
# az.legend()
# plt.tight_layout()
# plt.show()

print(f"Position Min max: x:({np.min(position[:,0])},{np.max(position[:,0])}), y:({np.min(position[:,1])},{np.max(position[:,1])}), z:({np.min(position[:,2])},{np.max(position[:,2])})")
print(f"Velocity Min max: x:({np.min(velocity[:bound_upper,0])},{np.max(velocity[:bound_upper,0])}), y:({np.min(velocity[:bound_upper,1])},{np.max(velocity[:bound_upper,1])}), z:({np.min(velocity[:bound_upper,2])},{np.max(velocity[:bound_upper,2])})")
print(f"Acceleration Min max: x:({np.min(acceleration[:bound_upper,0])},{np.max(acceleration[:bound_upper,0])}), y:({np.min(acceleration[:bound_upper,1])},{np.max(acceleration[:bound_upper,1])}), z:({np.min(acceleration[:bound_upper,2])},{np.max(acceleration[:,2])})")




