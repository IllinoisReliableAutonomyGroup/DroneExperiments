# self_state = [x y z vx vy vz ax ay az yaw yawrate quaternions]

######################################################################################################################################################

#!/usr/bin/env python3
import numpy as np
np.random.seed(1024)

import rospy
from std_msgs.msg import Bool
import numpy as np
import pickle5 as pickle
import argparse
from DroneExperiments.core.constants import dt
from DroneExperiments.ros.DroneInterface import Drone, distance
from DroneExperiments.C3M.models.model_navio2_numpy import get_controller_wrapper
from DroneExperiments.core.refs.gen_xref_uref_from_xyz import gen_xref_uref_from_xyz
from DroneExperiments.core.refs import gate_ref
from geometry_msgs.msg import PoseArray
from functools import partial
from threading import Lock
import time
import os

parser = argparse.ArgumentParser(description="")
parser.add_argument('--ulimit', type=float, default=2., help='max control cmd.')
parser.add_argument('--controller', type=str, help='filename to the pytorch model file.')
parser.add_argument('--save', type=str, help='filename to save the data.')
parser.add_argument('--oneshot', dest='oneshot', action='store_true')
parser.set_defaults(oneshot=False)
args = parser.parse_args()

new_xref = None
new_uref = None
new_ref_available = False
new_ref_lock = Lock()
current_state = None
gates = {}
gid = None

def new_gate_callback(data):
    global new_xref, new_uref, new_ref_available, new_ref_lock, gates, current_state,gid

    for gate in data.poses:
        gid = int(gate.orientation.w)
        gates[gid] = np.array([gate.position.x, gate.position.y, gate.position.z, gate.orientation.x, gate.orientation.y, gate.orientation.z])

    if len(gates) is not 1:
        return

    v = 1.
    before = 2.
    after = 1.5
    xyz = partial(gate_ref.xyz, [current_state[:3].copy(), list(gates.values())[0], v, before, after])
    print("Xref is",xyz)
    xref, uref, t = gen_xref_uref_from_xyz(xyz, T=None, dt=dt, translate=False)
    
    msg.data = True
    pub.publish(msg)

    with new_ref_lock:
        new_xref = xref
        new_uref = uref
        new_ref_available = True

from DroneExperiments.core.constants import xy_Tc, z_Tc
def gen_path(T=5, dt=0.001):
    downsample_mult = 50
    dt = dt / downsample_mult

    t = np.arange(0., T, dt)
    x = np.linspace(-2.2,1.9,len(t))
    y = np.zeros(len(t))
    z = np.ones(len(t))* 2

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

    uref = np.zeros([t.shape[0], 3])
    uref[:, 0] = jx * xy_Tc + ax
    uref[:, 1] = jy * xy_Tc + ay
    uref[:, 2] = jz * z_Tc + az

    xref = xref[::downsample_mult]
    uref = uref[::downsample_mult]
    t = t[::downsample_mult]

    return xref, uref


def main():
    global new_xref, new_uref, new_ref_available, new_ref_lock, current_state
    global target_points_x, target_points_y, target_points_z
    global msg, pub
    global gid

    target_points_x = [-2.6, -2.6, 1.9, 2.0]
    target_points_y = [3.1, -1.30, -1.7, 2.57]
    target_points_z = [1.5, 1.5, 1.5, 1.5]

    rospy.init_node('c3m_gate', anonymous=True)
 
    pub = rospy.Publisher('/gate_evasion_check', Bool, queue_size=1)
    msg = Bool()
    msg.data = False
    pub.publish(msg)

    drone = Drone()
    current_state = drone.state


    controller = get_controller_wrapper(args.controller)

    if not args.save:
        controller_name = os.path.basename(args.controller)[:-4]
        args.save = '../data/c3m_course_' + controller_name + '_' + str(time.time()) + '.pkl'
    print('Will save to ' + args.save)

    rate = rospy.Rate(1. / dt)
    
    while not rospy.is_shutdown() and drone.latest_vicon <= 0.:
        rate.sleep()

    print("passed")
    # plan a trajectory
    rospy.Subscriber('/gate_vision_position', PoseArray, new_gate_callback, queue_size=1)


    # start tracking
    refs = []
    data = []
    states = []
    xstars = []
    ustars = []
    controls = []
    ref_ids = []
    xref = None
    uref = None
    t = 0

    # xref, uref= gen_path()

    rospy.loginfo('Start tracking')

    drone.guided_mode()

    # drone.goto([-2.2,0,2])
    print("Finshed GOTO")

    # print("xref",xref)
    # print("uref",uref)
    print("enter PART II")
    while not rospy.is_shutdown() and new_xref is None:
        # xstar = xref[t, :]
        # ustar = uref[t, :]
        # u = controller(current_state, xstar, ustar)
        drone.set_acc(0.2,0,0, 0.)

        if current_state[0] >= 1.:
            break
        # print("set acc:",u[0], u[1], u[2])
        t += 1
        rate.sleep()
    print("end")
    return
    while not rospy.is_shutdown():
        print("operating")
        with new_ref_lock:
            if new_ref_available:
                if not args.oneshot or xref is None:
                    xref = new_xref
                    uref = new_uref
                    new_ref_available = False
                    refs.append([xref, uref])
                    t = 0
        if t >= xref.shape[0]:
            break
        
        # print("YAW ANGLE",gates.get(gid)[3] )
        # drone.set_acc(0,0,0 , gates.get(gid)[3])
        # rate.sleep()
        
        xstar = xref[t, :]
        ustar = uref[t, :]
        u = controller(current_state, xstar, ustar)
        u = np.clip(u, -1. * args.ulimit, args.ulimit)
        states.append(current_state.copy())
        xstars.append(xstar.copy())
        ustars.append(ustar.copy())
        controls.append(u.copy())
        ref_ids.append(len(refs) - 1)
        drone.set_acc(u[0], u[1], u[2], 0.)
        t += 1
        rate.sleep()

    rospy.loginfo('Finished tracking')

    for i in range(10):
        drone.disarm()
        rate.sleep()

    drone.loiter_mode()

    data.append([np.array(d) for d in [states, controls, ref_ids, xstars, ustars]])

    with open(args.save, 'wb') as handle:
        pickle.dump([data, dt, refs], handle, protocol=pickle.HIGHEST_PROTOCOL)

if __name__ == '__main__':
    main()