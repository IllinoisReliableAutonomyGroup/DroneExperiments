import cv2
import numpy as np

def countour_detection(img):

    img = cv2.medianBlur(img,5)
    img = cv2.GaussianBlur(img,(5,5),0)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    H_low = 0
    H_high = 179
    S_low= 130
    S_high = 247
    V_low= 121
    V_high = 255

    hsv_low = np.array([H_low, S_low, V_low], np.uint8)
    hsv_high = np.array([H_high, S_high, V_high], np.uint8)

    mask = cv2.inRange(hsv, hsv_low, hsv_high)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img, contours, -1, (0,255,0), 3)

    return contours, hierarchy

def box4points(img, center_of_blob):

    # Find rectangle diagonal
    max_len = 0
    max_len_corner1 = -1
    max_len_corner3 = 3
    for i in range(3):
        length = np.linalg.norm([blob_centers[3][0]-blob_centers[i][0],blob_centers[3][1]-blob_centers[i][1]])
        if length > max_len:
            max_len = length
            max_len_corner1 = i

    corner2 = -1
    corner4 = -1
    for i in range(4):
        if max_len_corner1 != i and max_len_corner3 != i and corner2 == -1:
            corner2 = i
            continue
        
        if max_len_corner1 != i and max_len_corner3 != i and corner2 != -1 and corner4 == -1:
            corner4 = i
    
    cv2.line(img,blob_centers[max_len_corner1], blob_centers[corner2],(0,255,0),3)
    cv2.line(img,blob_centers[max_len_corner1], blob_centers[corner4],(0,255,0),3)
    cv2.line(img,blob_centers[max_len_corner3], blob_centers[corner2],(0,255,0),3)
    cv2.line(img,blob_centers[max_len_corner3], blob_centers[corner4],(0,255,0),3)

    return img

def box3points(img, center_of_blob):
    # Find rectangle diagonal
    max_len = 0
    max_len_corner1 = -1
    max_len_corner3 = 0
    for i in range(1,3):
        length = np.linalg.norm([blob_centers[0][0]-blob_centers[i][0],blob_centers[0][1]-blob_centers[i][1]])
        if length > max_len:
            max_len = length
            max_len_corner1 = i

    corner2 = -1
    for i in range(3):
        if max_len_corner1 != i and max_len_corner3 != i and corner2 == -1:
            corner2 = i
            break
    
    cv2.line(img,blob_centers[max_len_corner1], blob_centers[corner2],(0,255,0),3)
    cv2.line(img,blob_centers[max_len_corner3], blob_centers[corner2],(0,255,0),3)

    return img
    
def box2points(img, center_of_blob):
    cv2.line(img,blob_centers[0], blob_centers[1],(0,255,0),3)

    return img

if __name__ == "__main__":
    # Read image
    img = cv2.imread("Red_circle/training_set1.png")
    contours, hierarchy = countour_detection(img)

    if contours is not None:
        compare_area = []
        for i in contours:
            compare_area.append([i, cv2.contourArea(i)])
        compare_area.sort(key = lambda x: x[1])

        if len(compare_area) >= 2:
            blob_centers = []
            for j in range(1,len(compare_area)+1):
                area = compare_area[-j][1]
                if area>800: 
                    x,y,w,h = cv2.boundingRect(compare_area[-j][0])	
                    img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    blob_center = ((2*x+w)//2,(2*y+h)//2)
                    img = cv2.circle(img, blob_center, 2, (0,255,0), 2)
                    blob_centers.append(blob_center)

            if len(blob_centers) == 4:
                img = box4points(img, blob_centers)
            elif len(blob_centers) == 3:
                img = box3points(img, blob_centers)
            elif len(blob_centers) == 3:
                img = box2points(img, blob_centers)
            else:
                pass


    cv2.imshow("img",img)
    cv2.waitKey(0)
