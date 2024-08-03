import numpy as np
from DroneExperiments.utils.transformations import quaternion_matrix

# downward
extrinsic = [0.04096058824303646, 0.8596926451923002, 0.5128097635484755, 0.016425010086555288, 0.06208309992380738, 0.06216159908025895, -0.03233385507632538]

# # forward
# extrinsic = [0.056084561187800405, 0.6995116501576115, 0.7114711778925265, 0.04116219992740323, -0.04499470115645796, 0.05659890257845717, -0.03833005447985708]

quaternion = extrinsic[:4]
translation = extrinsic[4:]
T_DC = quaternion_matrix(quaternion)
T_DC[:3, 3] = translation

def get_T_DC():
    return T_DC

def cam2drone(_P):
    P = np.ones_like(_P)
    P[:3] = _P
    return T_DC.dot(P)
