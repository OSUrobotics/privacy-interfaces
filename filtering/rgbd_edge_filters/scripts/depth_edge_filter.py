#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv, cv2
import numpy as np

bridge = CvBridge()
publisher = rospy.Publisher('/camera/depth_registered/hw_registered/image_rect_raw_sobel', Image)

def apply_sobel(image_msg):
    image_cv2 = bridge.imgmsg_to_cv2(image_msg, '16UC1')
    #from matplotlib import pyplot as plt
    #plt.plot(image_cv2[240, :], 'b')
    #image_cv2 = cv2.medianBlur(image_cv2, ksize=5)
    image_cv2_sobel = cv2.Sobel(image_cv2, ddepth=-1, dx=1, dy=1, ksize=5)
    #plt.plot(image_cv2_sobel[240, :],'r')
    #plt.show()
    upper = 1000.0
    image_cv2_sobel[image_cv2_sobel > upper] = upper  # clip
    image_cv2_sobel *= 65535.0/upper  # rescale to 16-bit
    image_cv2_sobel = cv2.convertScaleAbs(image_cv2_sobel, alpha=(255.0/65535.0))  # conversion to uint8
    image_cv2_sobel = 255 - image_cv2_sobel  # invert the image (8-bit)
    image_msg_sobel = bridge.cv_to_imgmsg(cv.fromarray(image_cv2_sobel), encoding='mono8')
    publisher.publish(image_msg_sobel)

def main():
    rospy.init_node('depth_edge_filter')
    rospy.Subscriber('/camera/depth_registered/hw_registered/image_rect_raw', Image, apply_sobel)
    rospy.spin()

if __name__ == '__main__':
    main()
