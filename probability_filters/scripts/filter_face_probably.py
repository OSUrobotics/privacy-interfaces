#!/usr/bin/env python
# 
# Detects face using OpenCV
# Tracks face using Kalman filter
# Filters face using (here's the novelty) a filter area that scales with uncertainty
# 

import rospy
import os
import sys
import rosbag
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from kalman_filter import Kalman 
import numpy
import cv2
from cv_bridge import CvBridge


class ImageHandler():
    """ Holds Image info; steps through frames """
    def __init__(self):
        self.image = None
        self.bridge = CvBridge()

    def open_bag(self, bag_path):
        self.bag = rosbag.Bag(bag_path)
        self.images = self.bag.read_messages(topics='/camera/rgb/image_color')
        
        # Prepare for UV <-> XYZ conversion
        camera_info = self.bag.read_messages(topics='/camera/rgb/camera_info').next()[1]
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info)

    def get_image(self):
        imgmsg = self.images.next()[1]
        self.image = self.bridge.imgmsg_to_cv2(imgmsg)
        

class FaceTracker():
    """ Keeps track of face location. """
    def __init__(self):
        self.classifier = cv2.CascadeClassifier(os.environ['ROS_ROOT'] + '/../OpenCV/haarcascades/haarcascade_frontalface_alt.xml')
        self.tracker = Kalman(Q=.002, R=0.01, P=.01)  # default values

    def detect_face(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.classifier.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            r = max(h, w)/2
            self.tracker.update([x + w/2, 
                                 y + h/2, 
                                 r])
            cv2.rectangle(image, (x,y), (x+w,y+h), (255,0,0), 2)
            cv2.circle(image, (x + w/2, y + h/2), 3, (0,255,0), -1)

        [u, v, r] = self.tracker.values()  # get estimates
        p = sum(self.tracker.p) / len(self.tracker.p)

        cv2.circle(image, (int(u), int(v)), 3, (0,0,255), -1)  # location estimate
        cv2.circle(image, (int(u), int(v)), int(r), (0,0,255), 2)  # location uncertainty
        cv2.imshow('Image with Face(s)', image)
        cv2.waitKey(10)

        return [u, v, r]
        
        
class ProbabilityFilter(ImageHandler, FaceTracker):
    """ Filters objects in images based on uncertain location data. """
    def __init__(self):
        ImageHandler.__init__(self)
        FaceTracker.__init__(self)

    def run_detect_face(self):
        [u, v, r] = self.detect_face(self.image)


def filter_face():
    """ Main function """
    rospy.init_node('filter_face')
    
    # Handle arguments
    if len(sys.argv) < 2:
        rospy.logerr('Must pass at least the first argument: the bag file path.')
        return -1
    else:
        bag_path = sys.argv[1] 
    
    #face_filter = ProbabilityFilter([640.0/2, 480.0/2])
    #face_filter = ProbabilityFilter([320, 100])
    face_filter = ProbabilityFilter()
    face_filter.open_bag(bag_path)

    while True:
        try:
            face_filter.get_image()
        except StopIteration:
            rospy.loginfo('Processed last image; exiting!')
            return 0
    
        face_filter.run_detect_face()

    # Update Kalman filter

    # Apply filter

    # Display image


if __name__ == "__main__":

    filter_face()
    
    cv2.destroyAllWindows()
