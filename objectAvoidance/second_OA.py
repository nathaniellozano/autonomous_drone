from scipy.spatial import distance as dist
import numpy as np
import cv2
import imutils
import tello_drone as tello
import time


host = ''
port = 9000
local_address = (host, port)

# Pass the is_dummy flag to run  on a local camera
drone = tello.Tello(host, port, is_dummy=False)
#drone = tello.Tello(host, port, is_dummy=True) #local camera



frame_read = drone.get_frame_read()



frame1 = frame_read.frame
frame2 = frame_read.frame

cap = drone.get_video_capture()
#functions for dodging 
#still need to test but for right now
#only move to right works
def dodge_right_center(x):
    drone.move_right(x)

#still need to test but for right now
#only move to left works
def dodge_left_center(x):
    drone.move_left(x)


#function for reaching goal
#still need to test but for right now
#only move forward works
def goal(x):
    drone.move_forward(x)



#works great with finding an object based on color
def hsv_Edge(frame):
    #yellow color detection range
    l_b = np.array([22,153,62])
    u_b = np.array([80,255,255])
    #testing other colors
    #l_b = np.array([51,51,00])
    #u_b = np.array([255,255,204])

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    mask =cv2.inRange(hsv,l_b,u_b)


    res = cv2.bitwise_and(frame, frame, mask=mask)

    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    edge = cv2.GaussianBlur(gray, (5,5), 0)
    _, edge = cv2.threshold(edge, 20, 255, cv2.THRESH_BINARY)


    return edge
#will keep since it does work for focusing on the object I want
def f_Marker( mark ):

    """
    finds largest contour/object among contours in frame
    ln#74-75
    returns minAreaRect and contourArea of largest object
    """

    contours, hierarchy = cv2.findContours(mark, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    
    cnts = [(cv2.contourArea(contour), contour) for contour in contours]

    if len(contours)==0:
        return 0, 0, None
    else:
        c = max(cnts, key=lambda x: x[0])[1]
        return cv2.minAreaRect( c ), cv2.contourArea(c), c
    
    

moved0=True
moved1=False
moved2=False
left= False
right =False
c=0


while True:
    edged = hsv_Edge(frame1)
    marker, area, cnt = f_Marker( edged )


    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)

    # Calculate frame center
    center_x = int(width/2)
    center_y = int(height/2)

    obj_center_x = center_x
    obj_center_y = center_y
    z_area = 0

    """
    if area from f_Marker is below threshold this case 1000 and found
    continues to be false rotate drone which acts as a search for object.
    """
    
    
    


    if area >= 5000 or moved1==True or moved2==True:

    # draw a bounding box around the image and display it
        if moved0 ==True or area>= 5000:

            box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
            box = np.int0(box)
            cv2.drawContours(frame1, [box], -1, (0, 255, 0), 2)

            M = cv2.moments(cnt)


            (startX, startY, endX, endY) = box


            obj_center_x = int((startX[0] + endX[0]) / 2.0)
            obj_center_y = int((startY[1] + endY[1]) / 2.0)
            z_area = (endX[1]+endY[1]) * (endX[0] + endY[0])

        # using  moments to calculate center ln. 255
            o_x = int(M['m10']/M['m00'])
            o_y = int(M['m01']/M['m00'])
            mz_area = M['m00']


        

        # Calculate recognized face offset from center
        # offset_x = obj_center_x - center_x
            offset_x = o_x - center_x
        # Add 30 so that the drone covers as much of the subject as possible
        # offset_y = obj_center_y - center_y - 30
            offset_y = o_y - center_y - 30





            cv2.putText(frame1, "Target Acquired:", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 3)
            cv2.imshow("feed", frame1)
        if moved1==True and area < 5000:
            time.sleep(3)
            moved1=False
            moved2=True
            drone.move_forward(120)
            time.sleep(3)
        elif moved2==True and area < 5000:
            if left==True:
                time.sleep(1)
                moved2=False
                drone.move_left(90)
                time.sleep(1)
            elif right==True:
                time.sleep(1)
                moved2=False
                drone.move_right(90)
                time.sleep(1)
        elif moved0==True or area >= 5000 and c<2:
            if not -90 <= offset_x <= 90 and offset_x is not 0:
                if offset_x < 0:
                    left=True
                    c+=1
                    moved0=False
                    moved1=True
                    dodge_right_center(90)
                    time.sleep(2)
                elif offset_x > 0:
                    right=True
                    c+=1
                    moved0=False
                    moved1=True
                    dodge_left_center(90)
                    time.sleep(2)
                else:
                    right=True
                    c+=1
                    moved0=False
                    moved1=True
                    dodge_left_center(90)
                    time.sleep(2)
            else:
                right=True
                c+=1
                moved0=False
                moved1=True
                dodge_left_center(90)
                time.sleep(2)
    else:
        goal(70)
    cv2.imshow("feed", frame1)
    cv2.imshow("edged",edged )

    frame1 = frame2
    frame2 = frame_read.frame

    if cv2.waitKey(1) == ord('q'): # Land the drone once q key is hit
        break




drone.end()
cv2.destroyAllWindows()