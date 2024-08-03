import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
import pickle
import argparse

# Generate trajectory in YZ Plane

def xyz(args, real_t):
   period, sizex, sizey = args
   print('period/sizex/sizey: ', period, sizex, sizey)

   # Load cubic splines from a file
   with open(f'trajectories_3D/traj_cubic_splines_0.pkl', 'rb') as f:
      splines = pickle.load(f)

   cs_x = splines['cs_x']
   cs_y = splines['cs_y']
   cs_z = splines['cs_z']

   # Define a range for interpolation
   print("realt",len(real_t))
   t_fine = np.linspace(0, len(cs_x.x) - 1, len(real_t))

   # Interpolate the spline fits

   y = cs_y(t_fine)
   z = cs_z(t_fine)

   x = np.ones_like(y) * 3.22

   return x, y, z
