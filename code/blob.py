#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import math
import numpy as np
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from quadcopter.msg import TargetInfo
from quadcopter.msg import Contour
from geometry_msgs.msg import Point32
from timeit import default_timer as timer
import atexit
import atexit

fourcc = cv2.VideoWriter_fourcc(*'MP4V')
out = cv2.VideoWriter('project_vision3.mp4', fourcc, 20.0, (1280,720))

# out = cv2.VideoWriter('project.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, (1280,720))

rospy.init_node('identifier_estimate', anonymous=True)
pub = rospy.Publisher('/landing_target_info_new', TargetInfo, queue_size=10)
now = rospy.get_time()

global info, flag_imageshow, cvFrame
cvFrame = np.zeros((500,500,3), np.uint8)
info = TargetInfo();
flag_imageshow=1

def exit_handler():
    out.release()



def segment_colour(frame,typ=1):
    hsv_roi =  cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    if typ==1:
        hsv_range = np.array([160,160,10,190,255,255])
        ycr_range = np.array([0.,165.,0.,255.,255.,255.])
    elif typ==2:
        hsv_range = np.array([20,41,133,45,255,255])
        ycr_range = np.array([210.,146.,16.,210.,146.,16.])
    else:
        hsv_range = np.array([0,0,30,180,10,70])
        ycr_range = np.array([0.,165.,0.,255.,255.,255.])
    mask_1 = cv2.inRange(hsv_roi, hsv_range[0:3], hsv_range[3:])
    ycr_roi=cv2.cvtColor(frame,cv2.COLOR_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, ycr_range[0:3], ycr_range[3:])
    if typ==1:
        mask= mask_1 | mask_2
    else:
        mask = mask_1;
    # mask = mask_1 | mask_2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    if typ ==1:
        mask= cv2.erode(mask,kern_erode)
        mask=cv2.dilate(mask,kern_dilate)
    return mask

def find_blob(blob):
    largest_contour=0
    cont_index=0
    _,contours,_= cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour=area
            cont_index=idx
    
    if len(contours) > 0:
        (x1,y1),radius  = cv2.minEnclosingCircle(contours[cont_index])
        x1 = round(x1,3)
        y1 = round(y1,3)
        detect=1
    else:
        x1 = 0
        y1 = 0
        radius = 0
        detect = 0
    return x1,  y1, radius, detect


def receiveimage(data):
    global info, flag_imageshow, cvFrame
    bridge=CvBridge()
    cvFrame = bridge.imgmsg_to_cv2(data,"passthrough")

def color_det(event):
    global info, flag_imageshow, cvFrame, now
    frame = cvFrame
    now = rospy.get_time()
    #cvFrame = frame
    #rate = rospy.Rate(20)

    bridge=CvBridge()
    mask_red = segment_colour(frame)
    if (mask_red ==0).all():
        mask_red = segment_colour(frame,3)
    mask_yellow = segment_colour(frame,2)
    x, y, radius,detect = find_blob(mask_red)
    x2, y2, radius2, detect2 = find_blob(mask_yellow)
    if (not detect):
        info.detected=0
        info.center.x=-1
        info.center.y=-1
        info.radius=-1

        info.detected2=0
        info.center2.x=-1
        info.center2.y=-1
        info.radius2=-1
    else:
        info.detected = 1
        info.center.x = x
        info.center.y = y
        info.radius = radius
        # rospy.loginfo('centre detected is %s'%([x,y]))
        if not (detect2):
            info.detected2 = 0
            info.center2.x = -1
            info.center2.y = -1
            info.radius2 = -1
        else:
            info.detected2 = 1
            info.center2.x = x2
            info.center2.y = y2
            info.radius2 = radius2
            # rospy.loginfo('centre2 detected is %s'%([x2,y2]))
        cv2.circle(frame, (int(x),int(y)), int(radius), (0, 255, 0), 2)
        # cv2.circle(frame, (int(x), int(y)), 3, (110, 0, 255), -1)
        # cv2.circle(frame, (int(x2),int(y2)), int(radius), (0, 255, 0), 2)
        # cv2.circle(frame, (int(x2), int(y2)), 3, (110, 0, 255), -1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (int(frame.shape[1]*0.8) ,int(frame.shape[0]*0.9))
        fontScale = 1
        fontColor = (255,255,255)
        lineType = int(2)
        cv2.putText(frame,str(now), 
            bottomLeftCornerOfText, 
            font, 
            fontScale,
            fontColor,
            lineType)

    info.time = now
    rospy.loginfo('flag detected is %s', info.detected)
    rospy.loginfo('flag detected2 is %s', info.detected2)
    # rospy.loginfo('time is %s', info.time)
    pub.publish(info)
    # imgarray.append(frame)

    out.write(frame)
    #re = cv2.resize(frame, (500, 500), interpolation = cv2.INTER_AREA) 
    
    #if flag_imageshow == 1:
    #    cv2.imshow('detection',re)
    #    cv2.waitKey(1)
    #k = cv2.waitKey(5) & 0xff
    #if k == 27:
    #    flag_imageshow = 0
    #    cv2.destroyAllWindows()
    #rate.sleep()

def listener():
    global imgarray
    imgarray=[]
    rospy.Subscriber('/camera_on_quad/images', Image, receiveimage)
    timer=rospy.Timer(rospy.Duration(20/1000.0),color_det)
    rospy.spin()

if __name__ == '__main__':
    try:
        atexit.register(exit_handler)
        listener()
    except rospy.ROSInterruptException:
        pass