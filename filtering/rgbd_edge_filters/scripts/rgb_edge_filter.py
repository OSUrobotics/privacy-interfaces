#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv, cv2

bridge = CvBridge()
publisher = rospy.Publisher('/camera/rgb/image_rect_color_sobel', Image)

def apply_sobel(image_msg):
    image_cv2 = bridge.imgmsg_to_cv2(image_msg, 'mono8')  # this also converts to grayscale
    image_cv2_sobel = cv2.Sobel(image_cv2, ddepth=-1, dx=1, dy=1, ksize=5)
    image_cv2_sobel = 255 - image_cv2_sobel  # invert the image
    image_msg_sobel = bridge.cv_to_imgmsg(cv.fromarray(image_cv2_sobel), encoding='mono8')
    publisher.publish(image_msg_sobel)

def main():
    rospy.init_node('rgb_edge_filter')
    rospy.Subscriber('/camera/rgb/image_rect_color', Image, apply_sobel)
    rospy.spin()

if __name__ == '__main__':
    main()
