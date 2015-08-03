#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge
import cv, cv2
import numpy
import matplotlib
import copy

from rosCV import rosCV as rcv

# import code # FOR TESTING

class Bound():
    def __init__(self, topic):
        # self.defaultManip = rospy.get_param('boundingBox/defaultManip')
        rospy.Subscriber(topic, Image, self.image_callback)
        rospy.Subscriber('camera/depth/image', Image, self.depth_callback)
        self.rcv = rcv()
        self.pub = rospy.Publisher(topic + '_filtered', Image)
    def depth_callback(self, image):
        self.depth_image = self.rcv.depthToCv2(image)
    def image_callback(self, image_in):
        """ Get image to which we're subscribed. """

        # Import and convert
        image_cv2 = self.rcv.toCv2(image_in)
        image_hsv = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2HSV)

        try:
            pink_lowerb = numpy.array((140, 100,100))
            pink_upperb = numpy.array((170,255, 255))
            pink_x, pink_y, pink_area = self.rcv.find_marker(image_cv2, pink_lowerb, pink_upperb)
            
            green_lowerb = numpy.array((50, 100,100))
            green_upperb = numpy.array((80,255, 255))
            green_x, green_y, green_area = self.rcv.find_marker(image_cv2, green_lowerb, green_upperb)

            special_area = image_hsv[pink_y:green_y, pink_x:green_x] 

            markings = image_cv2[pink_y:green_y, pink_x:green_x]
            markings = cv2.cvtColor(markings, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(markings, 10, 20)
            img_height = len(image_cv2)
            img_width = len(image_cv2[0])
            mask = numpy.zeros((img_height, img_width), dtype=numpy.uint8)
            mask[pink_y:green_y, pink_x:green_x] = edges
            kernel = numpy.ones((5,5),'uint8')
            mask = cv2.dilate(mask, kernel)
            # mask = cv2.erode(mask, kernel)
            board_depth = self.depth_image[pink_y, pink_x]
            # print "board depth = {0}".format(board_depth)
            # print self.depth_image
            # print numpy.where(self.depth_image <= board_depth - 0.2)
            # http://stackoverflow.com/questions/432112/is-there-a-numpy-function-to-return-the-first-index-of-something-in-an-array
            # for i in range(img_height):
            #     for j in range(img_width):
            #         if self.depth_image[i][j] <= board_depth - 0.25:
            #             mask[i][j] = 0

            image_cv2 = cv2.inpaint(image_cv2, mask, 5, cv2.INPAINT_TELEA)            
            # cv2.rectangle(image_cv2, (green_x, green_y), (pink_x, pink_y), (0, 0, 0), 3)
        except(ZeroDivisionError):
            pass
        # except(ZeroDivisionError, TypeError, AttributeError):
        self.rcv.imshow(self.depth_image)
        # self.rcv.imshow(image_cv2)

        # Convert back to ROS Image msg
        image_out = self.rcv.toRos(image_cv2)
        self.pub.publish(image_out)



if __name__ == '__main__':

    rospy.init_node('boundingBox')
    boundingBox = Bound('/camera/rgb/image_color')

    rospy.spin()
