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
    V_low= 125
    V_high = 255

    hsv_low = np.array([H_low, S_low, V_low], np.uint8)
    hsv_high = np.array([H_high, S_high, V_high], np.uint8)

    mask = cv2.inRange(hsv, hsv_low, hsv_high)

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(img, contours, -1, (0,255,0), 3)

    return contours, hierarchy

def box4points(img, blob_centers):

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

def box3points(img, blob_centers):
    
    # Compare y height to determine corner pair
    y1 = np.abs(blob_centers[0][1] - blob_centers[1][1])
    y2 = np.abs(blob_centers[0][1] - blob_centers[2][1])

    if y1 < y2:
        # Point 0 and 1 lie on same plane
        cv2.line(img,blob_centers[0], blob_centers[1],(0,255,0),3)

        # Determine which point connects to point 2
        x1 = np.abs(blob_centers[0][0] - blob_centers[2][0])
        x2 = np.abs(blob_centers[1][0] - blob_centers[2][0])

        if x1 < x2:
            cv2.line(img,blob_centers[0], blob_centers[2],(0,255,0),3)
        else:
            cv2.line(img,blob_centers[1], blob_centers[2],(0,255,0),3)
        

    else:
        # Point 0 and 2 lie on same plane
        cv2.line(img,blob_centers[0], blob_centers[2],(0,255,0),3)
        
        x1 = np.abs(blob_centers[0][0] - blob_centers[1][0])
        x2 = np.abs(blob_centers[2][0] - blob_centers[1][0])

        if x1 < x2:
            cv2.line(img,blob_centers[0], blob_centers[1],(0,255,0),3)
        else:
            cv2.line(img,blob_centers[2], blob_centers[1],(0,255,0),3)

    return img
    
def box2points(img, blob_centers):
    cv2.line(img,blob_centers[0], blob_centers[1],(0,255,0),3)

    return img

def main(cap):

    # Read until video is completed
    while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret == True:
            contours, hierarchy = countour_detection(frame)

            if contours is not None:
                # Pick most probable blob according to size, limited to 4 max
                compare_area = []
                for i in contours:
                    compare_area.append([i, cv2.contourArea(i)])
                compare_area.sort(key = lambda x: x[1])

                if len(compare_area) >= 2:
                    blob_centers = []
                    iterate_range = min(5,len(compare_area)+1)
                    for j in range(1,iterate_range):
                        area = compare_area[-j][1]
                        if area>800: 
                            x,y,w,h = cv2.boundingRect(compare_area[-j][0])	
                            frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                            blob_center = ((2*x+w)//2,(2*y+h)//2)
                            frame = cv2.circle(frame, blob_center, 2, (0,255,0), 2)
                            blob_centers.append(blob_center)

                    if len(blob_centers) == 4:
                        frame = box4points(frame, blob_centers)
                    elif len(blob_centers) == 3:
                        frame = box3points(frame, blob_centers)
                    # elif len(blob_centers) == 2:
                    #     frame = box2points(frame, blob_centers)
                    else:
                        pass
                    
            # Display the resulting frame
            cv2.imshow('Frame',frame)
        
            # Press Q on keyboard to  exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
            # Press P to Pause Video
            if cv2.waitKey(25) & 0xFF == ord('p'):
                while True:
                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        break
            
        # Break the loop
        else: 
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    cap = cv2.VideoCapture("Red_circle/red_dot_stream.mp4")
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")

    main(cap)
    