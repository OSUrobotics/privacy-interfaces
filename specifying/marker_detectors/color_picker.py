#! /usr/bin/env python2
# Modified from a GitHub Gist by user "trhura" on Oct 11, 2014 (https://gist.github.com/trhura/e69fca88bbd941c7024b) to work w/ the Kinect in ROS.
 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
 
colors = []
bridge = CvBridge()

def show_colors():
    minh = min(c[0] for c in colors)
    mins = min(c[1] for c in colors)
    minv = min(c[2] for c in colors)
    maxh = max(c[0] for c in colors)
    maxs = max(c[1] for c in colors)
    maxv = max(c[2] for c in colors)
        
    lb = [minh,mins,minv]
    ub = [maxh,maxs,maxv]
    print lb, ub
    
 
def on_mouse_click (event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONUP:
        colors.append(frame[y,x].tolist())
        show_colors()
 
def image_callback(image_msg):
    image_cv2 = bridge.imgmsg_to_cv2(image_msg)
    update_colors(image_cv2)

def update_colors(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
    if colors:
        cv2.putText(frame, str(colors[-1]), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 0), 2)
    cv2.imshow('frame', frame)
    cv2.setMouseCallback('frame', on_mouse_click, hsv)
    cv2.waitKey(1)

def main():
    rospy.init_node('color_picker')
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()
 
if __name__ == "__main__":
    main()
