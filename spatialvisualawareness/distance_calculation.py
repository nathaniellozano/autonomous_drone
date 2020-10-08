"""
HSV edging and coordinate systems were pulled from the previous AFS group
Additions include: Dodge, Re-center, goal reached
Code Reference: https://github.com/DrexelLagare/Senior-Design-1/blob/master/Object%20Tracking/target_A.py
"""
from scipy.spatial import distance as dist
import numpy as np
import cv2
import imutils
import tello_drone as tello
import time
import pandas as pd 
import matplotlib.pyplot as plt



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
def dodge_right_center(x):
    drone.move_right(x)

def dodge_left_center(x):
    drone.move_left(x)


#function for reaching goal
def goal(x):
    drone.move_forward(x)



#works great with finding an object based on color
def hsv_Edge(frame):
    #yellow color detection range
    l_b = np.array([22,153,62])
    u_b = np.array([80,255,255])

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
    
    

test = [[0] for i in range(100)]
count =0
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
    if area >= 1:

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)
        cv2.drawContours(frame1, [box], -1, (0, 255, 0), 2)

        M = cv2.moments(cnt)


        (startX, startY, endX, endY) = box


        obj_center_x = int((startX[0] + endX[0]) / 2.0)
        obj_center_y = int((startY[1] + endY[1]) / 2.0)
        z_area = (endX[1]+endY[1]) * (endX[0] + endY[0])
        #focal_length= (marker[1][0]*24.0)/8.0
        distance= (7.75*990)/marker[1][0]
        if count+10 <110:
            test[count-10]=distance
        #print(distance)
        #time.sleep(5)
        count= count+1
    
    cv2.imshow("feed", frame1)
    cv2.imshow("edged",edged )

    frame1 = frame2
    frame2 = frame_read.frame
    if cv2.waitKey(1) & 0xFF == ord('q'): # Land the drone once q key is hit
        break


   

    #if cv2.waitKey(1) == ord('q'): # Land the drone once q key is hit
    #    break




drone.end()
cv2.destroyAllWindows()
df = pd.DataFrame(columns=['A'])
for i in range(100):
    df = df.append({'A': test[i]}, ignore_index=True)
b=df.sem(axis=0)
ab=df.plot.hist(bins=100)
print(df)
print(b)
print(b*1.645)
plt.show()
