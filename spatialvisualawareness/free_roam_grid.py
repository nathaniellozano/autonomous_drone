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



host = ''
port = 9000
local_address = (host, port)

# Pass the is_dummy flag to run  on a local camera
#drone = tello.Tello(host, port, is_dummy=False)
drone = tello.Tello(host, port, is_dummy=True) #local camera

#drone.rotate_ccw(90)
#drone.rotate_cw(90)
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


#function for moving forward in the space(checks if the space infront of drone is taken, then moves or rotates)
#works great with finding an object based on color
def move_in_space(blocked,cardinal_direction,grid,positionx,positiony):
    if cardinal_direction== 'N':
        if grid[positionx][positiony-1]==0 and blocked==0:
            positionx=positionx
            positiony=positiony-1
            drone.move_forward(20)
        else:
            if cardinal_direction== 'N':
                cardinal_direction= 'E'
            elif cardinal_direction== 'E':
                cardinal_direction= 'S'
            elif cardinal_direction== 'S':
                cardinal_direction= 'W'
            elif cardinal_direction== 'W':
                cardinal_direction= 'N'
            drone.rotate_cw(90)
            grid[positionx][positiony-1]=1
    elif cardinal_direction== 'E':
        if grid[positionx+1][positiony]==0 and blocked==0:
            positionx=positionx+1
            positiony=positiony
            drone.move_forward(20)
        else:
            if cardinal_direction== 'N':
                cardinal_direction= 'E'
            elif cardinal_direction== 'E':
                cardinal_direction= 'S'
            elif cardinal_direction== 'S':
                cardinal_direction= 'W'
            elif cardinal_direction== 'W':
                cardinal_direction= 'N'
            drone.rotate_cw(90)
            grid[positionx+1][positiony]=1
    elif cardinal_direction== 'S':
        if grid[positionx][positiony+1]==0 and blocked==0:
            positionx=positionx
            positiony=positiony+1
            drone.move_forward(20)
        else:
            if cardinal_direction== 'N':
                cardinal_direction= 'E'
            elif cardinal_direction== 'E':
                cardinal_direction= 'S'
            elif cardinal_direction== 'S':
                cardinal_direction= 'W'
            elif cardinal_direction== 'W':
                cardinal_direction= 'N'
            drone.rotate_cw(90)
            grid[positionx][positiony+1]=1
    elif cardinal_direction== 'W':
        if grid[positionx-1][positiony]==0 and blocked==0:
            positionx=positionx-1
            positiony=positiony
            drone.move_forward(20)
        else:
            if cardinal_direction== 'N':
                cardinal_direction= 'E'
            elif cardinal_direction== 'E':
                cardinal_direction= 'S'
            elif cardinal_direction== 'S':
                cardinal_direction= 'W'
            elif cardinal_direction== 'W':
                cardinal_direction= 'N'
            drone.rotate_cw(90)
            grid[positionx-1][positiony]=1
    print("moving")
    return 0,cardinal_direction,grid,positionx,positiony
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
distance=0
blocked= 0
positionx=5
positiony=5
cardinal_direction= 'N'
#will setup grid
#1111111
#1000001
#1000001
#1000001
#1000001
#1000001
#1111111
grid = [[0]*7 for i in range(7)]
for i in range(7):
    for j in range(7):
        if i == 0:
            grid[i][j]=1
        elif i==6:
            grid[i][j]=1
        elif j==0:
            grid [i][j]=1
        elif j==6:
            grid [i][j]=1
#for changing direction when rotating clockwise
#
#if cardinal_direction== 'N':
#    cardinal_direction= 'E'
#elif cardinal_direction== 'E':
#    cardinal_direction= 'S'
#elif cardinal_direction== 'S':
#    cardinal_direction= 'W'
#elif cardinal_direction== 'W':
#    cardinal_direction= 'N'
#forward direction position
"""
if cardinal_direction== 'N':
    if grid[positionx][positiony-1]==0:
        positionx=positionx
        positiony=positiony-1
        drone.move_forward(40)
    else:
        if cardinal_direction== 'N':
            cardinal_direction= 'E'
        elif cardinal_direction== 'E':
            cardinal_direction= 'S'
        elif cardinal_direction== 'S':
            cardinal_direction= 'W'
        elif cardinal_direction== 'W':
            cardinal_direction= 'N'
elif cardinal_direction== 'E:'
    if grid[positionx+1][positiony]==0:
        positionx=positionx+1
        positiony=positiony
        drone.move_forward(40)
    else:
        if cardinal_direction== 'N':
            cardinal_direction= 'E'
        elif cardinal_direction== 'E':
            cardinal_direction= 'S'
        elif cardinal_direction== 'S':
            cardinal_direction= 'W'
        elif cardinal_direction== 'W':
            cardinal_direction= 'N'
elif cardinal_direction== 'S':
    if grid[positionx][positiony+1]==0:
        positionx=positionx
        positiony=positiony+1
        drone.move_forward(40)
    else:
        if cardinal_direction== 'N':
            cardinal_direction= 'E'
        elif cardinal_direction== 'E':
            cardinal_direction= 'S'
        elif cardinal_direction== 'S':
            cardinal_direction= 'W'
        elif cardinal_direction== 'W':
            cardinal_direction= 'N'
elif cardinal_direction== 'W':
    if grid[positionx-1][positiony]==0:
        positionx=positionx-1
        positiony=positiony
        drone.move_forward(40)
    else:
        if cardinal_direction== 'N':
            cardinal_direction= 'E'
        elif cardinal_direction== 'E':
            cardinal_direction= 'S'
        elif cardinal_direction== 'S':
            cardinal_direction= 'W'
        elif cardinal_direction== 'W':
            cardinal_direction= 'N'
""" 
#will be needing time.sleep() to make movements and process images
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


        #(startX, startY, endX, endY) = box


        #obj_center_x = int((startX[0] + endX[0]) / 2.0)
        #obj_center_y = int((startY[1] + endY[1]) / 2.0)
        #z_area = (endX[1]+endY[1]) * (endX[0] + endY[0])
        #focal_length= (marker[1][0]*24.0)/8.0
        #cv2.putText(frame1, "Dodge!!!", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
        #            1, (0, 0, 255), 3)
        #cv2.imshow("feed", frame1)
        distance= (7.5*990)/marker[1][0]
        if 23.5 <= distance <= 24.5:
            blocked=1
            cv2.putText(frame1, "Blocked!!", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 3)
            cv2.imshow("feed", frame1)
        """
        time.sleep(4)
        print("before move:")
        print(blocked,cardinal_direction,positionx,positiony)
        (blocked,cardinal_direction,grid,positionx,positiony)= move_in_space(blocked,cardinal_direction,grid,positionx,positiony)
        print("after move:")
        print(blocked,cardinal_direction,positionx,positiony)
        time.sleep(4)
        """

        #print(distance)
        #time.sleep(5)
        #if blocked ==0
    time.sleep(3)
    print("before move:")
    print(distance,blocked,cardinal_direction,positionx,positiony)
    (blocked,cardinal_direction,grid,positionx,positiony)= move_in_space(blocked,cardinal_direction,grid,positionx,positiony)
    print("after move:")
    print(distance,blocked,cardinal_direction,positionx,positiony)
    time.sleep(3)
    #elif blocked==1
    
    cv2.imshow("feed", frame1)
    cv2.imshow("edged",edged )

    frame1 = frame2
    frame2 = frame_read.frame
    


   

    if cv2.waitKey(1) & 0xFF == ord('q'): # Land the drone once q key is hit
        break




drone.end()
cv2.destroyAllWindows()