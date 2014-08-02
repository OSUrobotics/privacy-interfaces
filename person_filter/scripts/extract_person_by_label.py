#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy
import cv, cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters


class PersonExtracter():
    def __init__(self, image_topic, labels_topic):

        self.bridge = CvBridge()

        # Synchronized subscribers
        sub_rgb = message_filters.Subscriber(image_topic, Image)
        sub_labels = message_filters.Subscriber(labels_topic, Image)

        ts = message_filters.TimeSynchronizer([sub_rgb, sub_labels], 1)
        ts.registerCallback(self.callback)

    def callback(self, image, labels):
        # Prep the images
        try:
            image_cv = self.bridge.imgmsg_to_cv(image)  # to CvMat
            labels_cv = self.bridge.imgmsg_to_cv(labels)

            image_np = numpy.asarray(image_cv)  # to numpy
            labels_np = numpy.asarray(labels_cv)
        except CvBridgeError, e:
            print e

        # Apply the mask
        mask = numpy.logical_not(labels_np)  # "True" where there are NOT people
        for channel in range(3):  # R, G, and B
            image_np[:, :, channel] *= mask;

        # in case we need to publish it
        try:
            image = self.bridge.cv2_to_imgmsg(image_np)
        except CvBridgeError, e:
            print e
                
        cv2.imshow('Extracted people', image_np)
        cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node('extract_person_by_label')
    extracter = PersonExtracter('/camera/rgb/image_color/sync', 
                                '/camera/depth_registered/image_labeled_users/sync')

    rospy.spin()
    
