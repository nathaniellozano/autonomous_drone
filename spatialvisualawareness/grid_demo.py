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
drone = tello.Tello(host, port, is_dummy=False)
#drone = tello.Tello(host, port, is_dummy=True) #local camera

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

#use this variable if you want to change the distance the drone will move in the grid(centimeters)
#this will also determine the size of each block in the grid
# block length = drone movement distance
#blocks should be square
distance_in_grid= 20
#use this variable to change the value for the drone to turn(degrees)
#to change from clockwise to counter clockwise you must go into move_in_space
#clockwise = rotate_cw(x)
#counter clockwise = rotate_ccw(x)
degrees_to_turn=90
#function for moving forward in the space(checks if the space infront of drone is taken, then moves or rotates)
#works great with finding an object based on color
#return blocked,cardinal_direction,grid,positionx,positiony values
def move_in_space(blocked,cardinal_direction,grid,positionx,positiony):
    #will change direction from N to E and rotate if its path is blocked
    if cardinal_direction== 'N':
        if grid[positionx][positiony-1]==0 and blocked==0:
            positionx=positionx
            positiony=positiony-1
            drone.move_forward(distance_in_grid)
        else:
            if cardinal_direction== 'N':
                cardinal_direction= 'E'
            elif cardinal_direction== 'E':
                cardinal_direction= 'S'
            elif cardinal_direction== 'S':
                cardinal_direction= 'W'
            elif cardinal_direction== 'W':
                cardinal_direction= 'N'
            drone.rotate_cw(degrees_to_turn)
            grid[positionx][positiony-1]=1
            #will change direction from E to S and rotate if its path is blocked
    elif cardinal_direction== 'E':
        if grid[positionx+1][positiony]==0 and blocked==0:
            positionx=positionx+1
            positiony=positiony
            drone.move_forward(distance_in_grid)
        else:
            if cardinal_direction== 'N':
                cardinal_direction= 'E'
            elif cardinal_direction== 'E':
                cardinal_direction= 'S'
            elif cardinal_direction== 'S':
                cardinal_direction= 'W'
            elif cardinal_direction== 'W':
                cardinal_direction= 'N'
            drone.rotate_cw(degrees_to_turn)
            grid[positionx+1][positiony]=1
            #will change direction from S to W and rotate if its path is blocked
    elif cardinal_direction== 'S':
        if grid[positionx][positiony+1]==0 and blocked==0:
            positionx=positionx
            positiony=positiony+1
            drone.move_forward(distance_in_grid)
        else:
            if cardinal_direction== 'N':
                cardinal_direction= 'E'
            elif cardinal_direction== 'E':
                cardinal_direction= 'S'
            elif cardinal_direction== 'S':
                cardinal_direction= 'W'
            elif cardinal_direction== 'W':
                cardinal_direction= 'N'
            drone.rotate_cw(degrees_to_turn)
            grid[positionx][positiony+1]=1
            #will change direction from W to N and rotate 90 degrees if its path is blocked
    elif cardinal_direction== 'W':
        if grid[positionx-1][positiony]==0 and blocked==0:
            positionx=positionx-1
            positiony=positiony
            drone.move_forward(distance_in_grid)
        else:
            if cardinal_direction== 'N':
                cardinal_direction= 'E'
            elif cardinal_direction== 'E':
                cardinal_direction= 'S'
            elif cardinal_direction== 'S':
                cardinal_direction= 'W'
            elif cardinal_direction== 'W':
                cardinal_direction= 'N'
            drone.rotate_cw(degrees_to_turn)
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
#will build grid
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

#will be needing time.sleep() to make movements and process images
#uncomment to demo movement when object is at a predefined distance in front of the drone
#In this demo the drone will takeoff and will determine the distance between itself and the object in front of it
#For this demo you will need to have a yellow object in the color range below
#l_b = np.array([22,153,62])
#u_b = np.array([80,255,255])
#The object also needs a 8 inch width, length will not affect the calculation
#set the object to be 24 inches away from the drone
#then it will rotate clockwise 90 degrees and head forward 20 centimeters
#pressing q on keyboard will end the program
"""
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

    
    #if area from f_Marker is below threshold this case 1000 and found
    #continues to be false rotate drone which acts as a search for object.
    
    if area >= 1:

        box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        box = np.int0(box)
        cv2.drawContours(frame1, [box], -1, (0, 255, 0), 2)

        M = cv2.moments(cnt)


     
        distance= (7.5*990)/marker[1][0]
        if 23.5 <= distance <= 24.5:
            blocked=1
            cv2.putText(frame1, "Blocked!!", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 3)
            cv2.imshow("feed", frame1)
            time.sleep(3)
            drone.rotate_cw(90)
            time.sleep(3)
            drone.move_forward(20)
    
        
    
    cv2.imshow("feed", frame1)
    cv2.imshow("edged",edged )

    frame1 = frame2
    frame2 = frame_read.frame
    


   
    # Land the drone once q key is hit
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
"""


#uncomment to demo movement in a grid with no object
#drone's movement will be like the steps below
#before movement
#2 will be the drone
#1111111
#1000001
#1000001
#1000001
#1000001
#1000021
#1111111
#i=0
#1111111
#1000001
#1000001
#1000001
#1000021
#1000001
#1111111
#i=1
#1111111
#1000001
#1000001
#1000021
#1000001
#1000001
#1111111
#i=2
#1111111
#1000001
#1000021
#1000001
#1000001
#1000001
#1111111
#i=3
#1111111
#1000021
#1000001
#1000001
#1000001
#1000001
#1111111
#i=4
#1111111
#1000021
#1000001
#1000001
#1000001
#1000001
#1111111
#i=5
#1111111
#1000021
#1000001
#1000001
#1000001
#1000001
#1111111
#i=6
#1111111
#1000001
#1000021
#1000001
#1000001
#1000001
#1111111
#i=7
#1111111
#1000001
#1000001
#1000021
#1000001
#1000001
#1111111
#i=8
#1111111
#1000001
#1000001
#1000001
#1000021
#1000001
#1111111
#i=9
#1111111
#1000001
#1000001
#1000001
#1000001
#1000021
#1111111
"""
for i in range(10):
    time.sleep(4)
    (blocked,cardinal_direction,grid,positionx,positiony)= move_in_space(blocked,cardinal_direction,grid,positionx,positiony)
    print(positionx,positiony)
    time.sleep(4)
"""

drone.end()
cv2.destroyAllWindows()