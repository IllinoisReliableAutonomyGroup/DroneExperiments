import numpy as np
import pickle5 as pickle
from DroneExperiments.C3M.models.numpy_layers import *
import os
def construct_controller(model_file):
    num_dim_x = 3
    num_dim_u = 1

    effective_dim_start = 1
    effective_dim_end = 3
    print("Current path",os.getcwd())
    with open(model_file, 'rb') as handle:
        ck = pickle.load(handle)

    w1_0w = ck['w1_0w']
    w1_0b = ck['w1_0b']
    w1_1w = ck['w1_1w']
    w1_1b = ck['w1_1b']
    w2_0w = ck['w2_0w']
    w2_0b = ck['w2_0b']
    w2_1w = ck['w2_1w']
    w2_1b = ck['w2_1b']

    dim = effective_dim_end - effective_dim_start
    c = 3 * num_dim_x

    model_u_w1 = Sequential(
        Linear(w1_0w, w1_0b),
        Tanh(),
        Linear(w1_1w, w1_1b))

    model_u_w2 = Sequential(
        Linear(w2_0w, w2_0b),
        Tanh(),
        Linear(w2_1w, w2_1b))

    def _controller(x, xref, uref):
        x = np.array(x)
        xref = np.array(xref)
        xe = x - xref
        w1 = model_u_w1(np.concatenate([x[effective_dim_start:effective_dim_end],xref[effective_dim_start:effective_dim_end]])).reshape(-1, num_dim_x)
        w2 = model_u_w2(np.concatenate([x[effective_dim_start:effective_dim_end],xref[effective_dim_start:effective_dim_end]])).reshape(num_dim_u, -1)
        u = w2.dot(np.tanh(w1.dot(xe))) + uref
        return u
    return _controller

def get_controller_wrapper(pretrained_model_file_xy, pretrained_model_file_z=None):
    xy_controller = construct_controller(pretrained_model_file_xy)
    if pretrained_model_file_z is None:
        z_controller = xy_controller
    else:
        z_controller = construct_controller(pretrained_model_file_z)

    def _controller(x, xref, uref):
        x = np.array(x)
        xref = np.array(xref)
        uref = np.array(uref)
        idx = 0
        ux = xy_controller(x[[0+idx,3+idx,6+idx]], xref[[0+idx,3+idx,6+idx]], uref[[idx]])
        idx = 1
        uy = xy_controller(x[[0+idx,3+idx,6+idx]], xref[[0+idx,3+idx,6+idx]], uref[[idx]])
        idx = 2
        uz = xy_controller(x[[0+idx,3+idx,6+idx]], xref[[0+idx,3+idx,6+idx]], uref[[idx]])
        return np.array([ux, uy, uz])

    return _controller
