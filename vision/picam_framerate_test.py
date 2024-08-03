import numpy as np
import cv2
import cv2.aruco as aruco
import math
import time
# from eulerconvert import rotationMatrixToEulerAngles
cap = cv2.VideoCapture(0)

img_width = 640
img_height = 480
frame_rate = 60
cap.set(2, img_width)
cap.set(4, img_height)
cap.set(5, frame_rate)

frame_id =0
while True:
    t1 = time.time()
    ret, frame = cap.read()
    # cv2.imshow("frame",frame)
   
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'): break

    # cv2.imwrite("frame_%d_rate60.jpg"%frame_id,frame)
    frame_id += 1


cap.release()
cv2.destroyAllWindows()
