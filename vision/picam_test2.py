import numpy as np
import cv2
import cv2.aruco as aruco
import math
import time
# from eulerconvert import rotationMatrixToEulerAngles

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

camera_mtx = np.loadtxt("calibration_cammatrix.out")
distortion_param = np.loadtxt("calibration_distparam.out")

marker_size = 187 #mm

cap = cv2.VideoCapture(0)

img_width = 640
img_height = 480
frame_rate = 40
cap.set(2, img_width)
cap.set(4, img_height)
cap.set(5, frame_rate)


while True:
    t1 = time.time()
    ret, frame = cap.read()
    t2 = time.time()
    #frame = cv2.flip(frame,0)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #gray = cv2.flip(gray,0)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    t3 = time.time()
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    t4 = time.time()
    realworld_tvec = [0,0,0]
    pitch, roll, yaw = 0, 0, 0
    t5 = t4
    t6 = t4
    t7 = t4
    if ids is not None:
        #aruco.drawDetectedMarkers(frame, corners)

        rvec, tvec, objpoints = aruco.estimatePoseSingleMarkers(corners,marker_size, camera_mtx, distortion_param)
        t5 = time.time()
        # Unpacking the compounded list structure
        rvec = rvec[0][0]
        tvec = tvec[0][0]
    
        aruco.drawAxis(frame, camera_mtx, distortion_param, rvec, tvec, 100)

        rvec_flip = rvec * -1
        tvec_flip = tvec * -1
        rotation_mtx, jacobian = cv2.Rodrigues(rvec_flip)
        realworld_tvec = np.dot(rotation_mtx, tvec_flip)
        print("tvec original", tvec, "realworld",realworld_tvec)

        pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_mtx)
        print("quaternions",math.degrees( pitch),math.degrees( roll), math.degrees(yaw))
        t6 = time.time()

        # Factor in pitch direction of camera angle pointing 45 degree downwards
        # pitch += 45*(np.pi/180)
    tf = time.time()
    # print('read: %.3fs, prep: %.3fs, det: %.3fs, parse: %.3fs, other: %.3fs, total: %.3fs'%(t2-t1, t3-t2, t4-t3, t5-t4, t6-t5, tf-t1))
    frame = cv2.flip(frame,0)
    #print("frame size",frame.shape)
    tvec_str = "real x=%3.0f y =%3.0f z=%3.0f"%(realworld_tvec[0],realworld_tvec[1],realworld_tvec[2])
    cv2.putText(frame, tvec_str, (20,430), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2, cv2.LINE_AA)
    angles_str = "pitch=%3.0f roll=%3.0f yaw=%3.0f"%(math.degrees(pitch), math.degrees(roll), math.degrees(yaw))
    cv2.putText(frame, angles_str, (20,460), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2, cv2.LINE_AA)
    cv2.imshow("frame",frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break
    if key == ord('c'):
        cv2.imwrite("livecap_blank.jpg",frame)


cap.release()
cv2.destroyAllWindows()
