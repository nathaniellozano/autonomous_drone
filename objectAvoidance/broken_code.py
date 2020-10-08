from scipy.spatial import distance as dist
import numpy as np
import cv2
import imutils
import tello_drone as tello
import time


host = ''
port = 9000
local_address = (host, port)

# Pass the is_dummy flag to run the face  detection on a local camera
drone = tello.Tello(host, port, is_dummy=False)




frame_read = drone.get_frame_read()



frame1 = frame_read.frame
frame2 = frame_read.frame

cap = drone.get_video_capture()
def dodge_right():
    drone.move_right(20)

def dodge_left():
    drone.move_left(20)


def goal(x):
    drone.move_forward(x)


def adjust_tello_position(offset_x, offset_y, offset_z):

    """
    Adjusts the position of the tello drone based on the offset values given from the frame
    :param offset_x: Offset between center and face x coordinates
    :param offset_y: Offset between center and face y coordinates
    """






    if not -90 <= offset_x <= 90 and offset_x is not 0:
        if offset_x < 0:
            dodge_right()
        elif offset_x > 0:
            dodge_left()

    



def hsv_Edge(frame):
    #yellow color detection range
    l_b = np.array([22,153,62])
    u_b = np.array([80,255,255])


    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    mask =cv2.inRange(hsv,l_b,u_b)


    res = cv2.bitwise_and(frame, frame, mask=mask)


#commented transformations caused tracking without movement
    # diff = cv2.absdiff(mask1, mask2)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    edge = cv2.GaussianBlur(gray, (5,5), 0)
    _, edge = cv2.threshold(edge, 20, 255, cv2.THRESH_BINARY)

    # edge = cv2.dilate(edge, None, iterations=2)

    return edge

def f_Marker( mark ):

    """
    finds largest contour/object among contours in frame
    ln#74-75
    returns minAreaRect and contourArea of largest object
    """

    contours, hierarchy = cv2.findContours(mark, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # if len(contours) > 0:
    #     cnts = imutils.grab_contours( contours )
    #     c = max(cnts, key = cv2.contourArea)
    cnts = [(cv2.contourArea(contour), contour) for contour in contours]

    if len(contours)==0:
        return 0, 0, None
    else:
        c = max(cnts, key=lambda x: x[0])[1]
        return cv2.minAreaRect( c ), cv2.contourArea(c), c
    # else:
    #     return False

found = False

"""
This loop will crash when the object moves out of screen
updates have been so that this is no longer a problem
"""

while True:




# testing : getting biggest contour and bounding box to it
# next step will be to put rotate drone if condition not met.

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

   
    
    
    if area < 1000 and found == False:
        goal(70)





    else:

    # draw a bounding box around the image and display it

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
        found = True

        if found == True:
            adjust_tello_position( offset_x, offset_y, mz_area)
                    

    cv2.imshow("feed", frame1)
    cv2.imshow("edged",edged )

    frame1 = frame2
    frame2 = frame_read.frame


    if cv2.waitKey(1) == ord('q'): # Land the drone once q key is hit
        break




drone.end()
cv2.destroyAllWindows()