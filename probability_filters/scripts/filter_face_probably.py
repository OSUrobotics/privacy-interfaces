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
        self.tracker = Kalman(Q=.002, R=0.01, P=.01) 

    def detect_face(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.classifier.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            self.tracker.update([x + w/2, 
                                 y + h/2, 
                                 w, 
                                 h])
            cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 1)
            cv2.circle(image, (x + w/2, y + h/2), 3, (0,255,0), -1)

        [u, v, w, h] = self.tracker.values()  # get estimates
        p = sum(self.tracker.p) / len(self.tracker.p)

        cv2.circle(image, (int(u), int(v)), 3, (0,0,255), -1)  # location estimate
        cv2.rectangle(image, (int(u - w/2), int(v - h/2)), (int(u + w/2), int(v + h/2)), (0,0,255), 1)  # location uncertainty
        cv2.imshow('Image with Face(s)', image)
        cv2.waitKey(10)

        return [u, v, w, h]
        
        
class ProbabilityFilter(ImageHandler, FaceTracker):
    """ Filters objects in images based on uncertain location data. """
    def __init__(self, bag_path):
        ImageHandler.__init__(self)
        FaceTracker.__init__(self)
        self.bag_out = rosbag.Bag(bag_path, 'w')
        self.topic_image = 'image'
        self.topic_mask = 'mask'

    def run_detect_face(self):
        self.face = []
        [u, v, w, h] = self.detect_face(self.image)
        self.face.append(u - w/2)
        self.face.append(v - h/2)
        self.face.append(u + w/2)
        self.face.append(v + h/2)
        self.mask = self.make_mask(self.image)

    def make_mask(self, image):
        mask = numpy.zeros(image.shape)
        mask[self.face[1] : self.face[3],
                  self.face[0] : self.face[2], :] = 255
        mask = mask.astype('uint8')
        return mask

    def write_face(self):
        image_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
        mask_msg = self.bridge.cv2_to_imgmsg(self.mask, "bgr8")
        self.bag_out.write(self.topic_image, image_msg)
        self.bag_out.write(self.topic_mask, mask_msg)
        rospy.loginfo('Wrote a frame to bag file!')


def filter_face():
    """ Main function """
    rospy.init_node('filter_face')
    
    # Handle arguments
    if len(sys.argv) < 2:
        rospy.logerr('Must pass at least the first argument: the INPUT bag file path.')
        return -1
    if len(sys.argv) < 3:
        rospy.logerr('Must pass at least the second argument: the OUTPUT bag file path.')
        return -1
    else:
        path_bag_in = sys.argv[1] 
        path_bag_out = sys.argv[2] 

    # Initialize object(s)
    #face_filter = ProbabilityFilter([640.0/2, 480.0/2])
    #face_filter = ProbabilityFilter([320, 100])
    face_filter = ProbabilityFilter(path_bag_out)
    face_filter.open_bag(path_bag_in)

    while True:
        try:
            face_filter.get_image()
            face_filter.run_detect_face()
            face_filter.write_face()
        except StopIteration:
            rospy.loginfo('Processed last image; exiting!')
            return 0


if __name__ == "__main__":

    filter_face()
    
    cv2.destroyAllWindows()
