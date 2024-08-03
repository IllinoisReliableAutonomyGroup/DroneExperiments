import numpy as np
import cv2
import cv2.aruco as aruco
import math
import time
# from eulerconvert import rotationMatrixToEulerAngles

scale = 1.0

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6
 
# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
 
    assert(isRotationMatrix(R))
 
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
 
    singular = sy < 1e-6
 
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
 
    return np.array([x, y, z])

camera_mtx = np.array([[7.158079776703193602e+02, 0.000000000000000000e+00, 3.210743294457899992e+02],
                       [0.000000000000000000e+00, 7.158209186456878115e+02, 2.792629023959584060e+02],
                       [0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00]])

camera_mtx = camera_mtx * scale
camera_mtx[2,2] = 1.

distortion_param = np.array([-3.977999898520637179e-01, 4.642948805087612069e-02, -6.796259068119826427e-03, 3.443550466207846283e-03, -5.915581008021588705e-01])

def detect_aruco(cap=None, save=None, visualize=True, marker_size=187):
    def cleanup_cap():
        pass
    if cap is None:
        cap = get_camera()
        cleanup_cap = lambda: cap.release()
    def cleanup():
        cleanup_cap()

    ret, frame = cap.read()
    width = int(frame.shape[1] * scale)
    height = int(frame.shape[0] * scale)
    dim = (width, height)
    frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters_create()
    corners, _ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    Ts = []
    ids = []
    if _ids is not None:
        rvecs, tvecs, objpoints = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_mtx, distortion_param)
        for i in range(len(_ids)):
            # Unpacking the compounded list structure
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]
            # print("Rvec",rvec)
            # print("Tvec",tvec)
            ids.append(_ids[i][0])
            if save or visualize:
                aruco.drawAxis(frame, camera_mtx, distortion_param, rvec, tvec, 100)

            rotation_mtx, jacobian = cv2.Rodrigues(rvec)
            translation = tvec
            T = np.identity(4)
            T[:3, :3] = rotation_mtx
            T[:3, 3] = translation / 1000.
            Ts.append(T)

    if save:
        cv2.imwrite(save, frame)

    if visualize:
        cv2.imshow("camera view", frame)

    cleanup()

    # Multiple ids in 1D list
    # Mutiple Ts, select first marker 2d array by Ts[0]
    return Ts, ids

def get_camera():
    cap = cv2.VideoCapture(0)

    img_width = 640
    img_height = 480
    frame_rate = 60
    cap.set(2, img_width)
    cap.set(4, img_height)
    cap.set(5, frame_rate)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap

def release_camera(cap):
    cap.release()

if __name__ == "__main__":
    cam = get_camera()
    while True:
        s = time.time()
        Ts, ids = detect_aruco(cam, visualize=True)
        print('run time %.3fs'%(time.time() - s))
        print("IDs:   ",ids)
        print("TS is:   ",Ts)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): break
    cv2.destroyAllWindows()
    release_camera(cam)
