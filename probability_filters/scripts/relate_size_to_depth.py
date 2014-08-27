#!/usr/bin/env python
# 
# Detects a series of faces at different depths
# to estimate a relation between face size and depth.
# Ultimate goal: window_expansion = f(dt, depth(face_size), confidence, camera_info)

import rospy
import os
import sys
from sensor_msgs.msg import Image, CameraInfo
import numpy
import cv2
from cv_bridge import CvBridge
import pickle  # gonna need a pickle

class ImageHandler():
    """ Holds Image msgs """
    def __init__(self, image_topic):
        self.have_image = False
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(image_topic, Image, self.handle_image)

    def handle_image(self, imgmsg):
        self.image = self.bridge.imgmsg_to_cv2(imgmsg)
        if not self.have_image:
            self.have_image = True
        

class FaceDetector():
    """ Measures face location, compiles important info. """
    def __init__(self):
        self.classifier = cv2.CascadeClassifier(os.environ['ROS_ROOT'] + '/../OpenCV/haarcascades/haarcascade_frontalface_alt.xml')
        self.data = []

    def detect_face(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.classifier.detectMultiScale(gray, 1.3, 5)
        if len(faces) > 0:
            (x,y,w,h) = faces[0]
            face = [x + w/2, y + h/2, w, h]
            cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 1)  # plot detected face
            depth = input('What depth is this? Report in meters: ')
            self.data.append([depth, face, image])

        else:
            rospy.loginfo('Detection failed! Try again.')
            return False

        cv2.imshow('Image with Face(s)', image)
        cv2.waitKey(0)

        return True

        
if __name__ == "__main__":

    rospy.init_node('relate_size_to_depth')

    # Handle arguments
    if len(sys.argv) < 2:
        rospy.logwarn('Defaulting OUTPUT file path to "default.pickle"')
        path_out = 'default.pickle'
    else:
        path_out = sys.argv[1] 

    image_handler = ImageHandler('/camera/rgb/image_color')
    face_detector = FaceDetector()

    while not image_handler.have_image:  # wait for first image
        rospy.sleep(0.01)

    N = input('Input number of measurements: ')
    for i in range(N):
        success = False
        while not success:
            feedback = raw_input('Press ENTER when subject gives thumbs-up, or type SKIP to move on.  ')
            if feedback == 'SKIP':
                rospy.loginfo('Skipping measurement!')
                break
            else:
                success = face_detector.detect_face(image_handler.image)

            if success:
                rospy.loginfo('Success! Moving to next measurement.')

    with open(path_out, 'w') as f:
        pickle.dump(face_detector.data, f)
        rospy.loginfo('Pickling successful! All done.')
    
    cv2.destroyAllWindows()
