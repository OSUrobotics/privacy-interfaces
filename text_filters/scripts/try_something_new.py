#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
import cv, cv2
import numpy
from sensor_msgs.msg import Image


bridge = CvBridge()
pub = rospy.Publisher("/image_out", Image)

def image_callback(image):
    """ Applies a new filter to the image and displays the result. """
    image_cv = bridge.imgmsg_to_cv(image)
    image_cv2 = numpy.asarray(image_cv)
    
    # Downsample the grayscale image
    gray = image_cv2[:, :, 0]/3 + image_cv2[:, :, 1]/3 + image_cv2[:, :, 2]/3
    gray = cv2.pyrDown(gray)
    #gray = cv2.pyrDown(gray)
    #gray = cv2.pyrDown(gray)

    # Make new 3-channel image
    image_new = numpy.zeros((gray.shape[0], gray.shape[1], image_cv2.shape[2]))
    image_new[:, :, 0] = image_new[:, :, 1] = image_new[:, :, 2] = gray
    image_new = image_new.astype('uint8')
    print image_new.shape

    # Re-publish
    image.data = bridge.cv_to_imgmsg(cv.fromarray(image_new), 
                                     encoding=image.encoding).data
    image.width = image_new.shape[1]
    image.height = image_new.shape[0]
    image.step = image.width * 3
    pub.publish(image)


if __name__ == "__main__":
    rospy.init_node("new_filter")
    rospy.Subscriber("/camera/rgb/image_color", Image, image_callback)
    rospy.spin()
