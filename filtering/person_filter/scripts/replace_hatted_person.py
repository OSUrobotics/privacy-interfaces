#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy
import cv, cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
from std_msgs.msg import Int8


class PersonExtracter():
    def __init__(self, image_topic, labels_topic, image_topic_unsynced):

        self.bridge = CvBridge()

        self.background = None
        self.image_count = 0
        self.hatted_user_id = 0

        self.pub = rospy.Publisher(image_topic_unsynced+'/filtered', Image)

        # UN-synchronized image subscriber (for background image)
        self.sub_unsync = rospy.Subscriber(image_topic_unsynced, Image, self.background_callback)
        while self.image_count < 10:
            rospy.sleep(0.1)
        self.sub_unsync.unregister()

        # Hatted user ID subscriber
        rospy.Subscriber('/hatted_head', Int8, self.hatted_callback)

        # Synchronized subscribers
        sub_rgb = message_filters.Subscriber(image_topic, Image)
        sub_labels = message_filters.Subscriber(labels_topic, Image)

        ts = message_filters.TimeSynchronizer([sub_rgb, sub_labels], 1)
        ts.registerCallback(self.main_callback)

    def background_callback(self, image):
        # Initialize the background
        if self.image_count == 9:
            image_cv = self.bridge.imgmsg_to_cv(image)  # to CvMat
            image_np = numpy.asarray(image_cv)  # to numpy
            self.background = image_np
            rospy.loginfo('Got background image!')
        self.image_count += 1


    def hatted_callback(self, hatted_user_id):
        self.hatted_user_id = hatted_user_id.data
 
    def main_callback(self, image, labels):
        # Prep the images
        try:
            image_cv = self.bridge.imgmsg_to_cv(image)  # to CvMat
            labels_cv = self.bridge.imgmsg_to_cv(labels)

            image_np = numpy.asarray(image_cv)  # to numpy
            labels_np = numpy.asarray(labels_cv)
        except CvBridgeError, e:
            print e

        # Make the mask
        if self.hatted_user_id != 0:
            mask = labels_np != self.hatted_user_id  # "True" everywhere but the hatted person
            
            # Apply the mask
            for channel in range(3):  # R, G, and B
                image_np[:, :, channel] *= mask;
                image_np[:, :, channel] += self.background[:, :, channel] * numpy.logical_not(mask)

        # in case we need to publish it
        try:
            image = self.bridge.cv2_to_imgmsg(image_np, encoding='bgr8')
        except CvBridgeError, e:
            print e
                
        cv2.imshow('Extracted people', image_np)
        cv2.waitKey(1)

        self.pub.publish(image)


if __name__ == "__main__":
    rospy.init_node('extract_person_by_label')
    extracter = PersonExtracter('/camera/rgb/image_color/sync', 
                                '/camera/depth_registered/image_labeled_users/sync',
                                '/camera/rgb/image_color')

    rospy.spin()
    
