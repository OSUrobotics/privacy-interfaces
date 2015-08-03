#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv, cv2
import numpy
from sensor_msgs.msg import Image
import sys


bridge = CvBridge()
pub = rospy.Publisher("/image_out", Image)
shift_by = 128

def image_callback(image):
    """ Shifts bit(s) from image and displays the result. """
    image_cv = bridge.imgmsg_to_cv(image)
    image_cv2 = numpy.asarray(image_cv)
    image_cv2 = cv2.blur(image_cv2, (5, 5))
    gray = image_cv2[:, :, 0]/3 + image_cv2[:, :, 1]/3 + image_cv2[:, :, 2]/3
    image_cv2[:, :, 0] = image_cv2[:, :, 1] = image_cv2[:, :, 2] = gray
    image_cv2 = image_cv2 // shift_by * shift_by
    image.data = bridge.cv_to_imgmsg(cv.fromarray(image_cv2), 
                                     encoding=image.encoding).data
    pub.publish(image)


if __name__ == "__main__":
    rospy.init_node("bit_shifter_gray")
    shift_by = int(sys.argv[1])
    rospy.loginfo('Shifting by a factor of {0}'.format(shift_by))
    rospy.Subscriber("/camera/rgb/image_color", Image, image_callback)
    rospy.spin()
