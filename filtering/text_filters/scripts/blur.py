#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv, cv2
import numpy
from sensor_msgs.msg import Image


bridge = CvBridge()
pub = rospy.Publisher("/image_out", Image)

def image_callback(image):
    """ Blurs the image and displays the result. """
    image_cv = bridge.imgmsg_to_cv(image)
    image_cv2 = numpy.asarray(image_cv)
    image_cv2 = cv2.blur(image_cv2, (10, 10))
    image.data = bridge.cv_to_imgmsg(cv.fromarray(image_cv2), 
                                     encoding=image.encoding).data
    pub.publish(image)


if __name__ == "__main__":
    rospy.init_node("blur")
    rospy.Subscriber("/camera/rgb/image_color", Image, image_callback)
    rospy.spin()
