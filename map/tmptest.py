from DroneExperiments.vision.detect_aruco import detect_aruco, get_camera, release_camera
from DroneExperiments.vision.cam2drone import get_T_DC
T_DC = get_T_DC()

import numpy as np
camera = get_camera()
Ts, ids = detect_aruco(camera, visualize=False)

pose_C = Ts[0]
pose_D = T_DC.dot(pose_C)
pose = np.linalg.inv(pose_D)

print("POSEC",pose_C)
print("POSE_D",pose_D)
print("POSE",pose)