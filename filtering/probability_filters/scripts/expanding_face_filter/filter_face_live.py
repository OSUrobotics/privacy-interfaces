#!/usr/bin/env python
# 
# Expands the filter area when the face detector fails.
# 
# Makes these assumptions:
#  a) Mean face size is 187.5mm (see mean tr-gn distance for NAW males in Farkas et al., 2005)
#  b) Mean walking speed is 162.6cm/s, SD is 20.1cm/s (see mean "fast" gait speed for 20-29 y.o. men in Oberg et al., 1993)
# 
# Outline of method for obscuring undetected faces:
#  1) Start with the last detected face window; we're going to expand it
#  2) Use ratio of detected face height (pixels) to mean face height (mm) to convert walking speeds (cm/s) to pixels/second
#  3) Multiply by seconds between frames ("dt") to get mean & SD of expansion in pixels
#  4) Apply confidence level to get desired expansion in pixels
#  5) Display expanded region


import rospy
import os
import sys
import rosbag
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
#from pykalman import KalmanFilter
import numpy
import cv2
from cv_bridge import CvBridge


class FaceDetector():
    """ Measures face location, compiles important info. """
    def __init__(self, info_path, image_topic):
        # Create depth model for Asus Xtion camera
        bag = rosbag.Bag(info_path)
        infos = bag.read_messages(topics='/camera/rgb/camera_info')
        info = infos.next()[1]
        self.model = PinholeCameraModel()
        self.model.fromCameraInfo(info)

        # Ready the face detector
        self.classifier = cv2.CascadeClassifier(os.environ['ROS_ROOT'] + '/../OpenCV/haarcascades/haarcascade_frontalface_alt.xml')
        self.have_face = False

        # Subscribe to images
        self.have_image = False
        self.have_t_ref = False
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(image_topic, Image, self.handle_image)

    def handle_image(self, imgmsg):
        if not self.have_t_ref:
            self.t_ref = imgmsg.header.stamp.to_sec()
            self.t_old = 0.0
            self.have_t_ref = True
        t_new = imgmsg.header.stamp.to_sec() - self.t_ref
        dt = t_new - self.t_old
        self.t_old = t_new
            
        image = self.bridge.imgmsg_to_cv2(imgmsg)
        if not self.have_image:
            self.have_image = True
        success = self.detect_face(image, dt)

    def detect_face(self, image, dt):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.classifier.detectMultiScale(gray, 1.15, 1)

        if len(faces) > 0:
            (x,y,w,h) = faces[0]
            self.face = [x, y, w, h]
            self.face_size = w
            cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 1)  # plot detected face
            if not self.have_face:
                self.have_face = True
        else:
            if self.have_face:
                x,y,w,h = self.expand_filter(1, dt)
                cv2.rectangle(image, (x,y), (x+w,y+h), (255,0,255), 3)  # plot expanded filter area
                self.face = [x, y, w, h]

        cv2.imshow('Image with Face', image)
        cv2.waitKey(1)

    def expand_filter(self, ci, dt):
        """ Input: confidence interval N as in N*SD or N-sigma. """
        (x, y, w, h) = self.face  # break it down
        mean_face = 0.1875  # m
        mean_speed = 1.626  # m/s
        sd_speed = 0.201   # m/s
        meters_to_pixels = self.face_size / mean_face
        print meters_to_pixels
        mean_speed *= meters_to_pixels  # convert
        sd_speed *= meters_to_pixels  # convert
        expansion = (mean_speed + ci * sd_speed) * dt
        print mean_speed, sd_speed
        print dt, expansion
        print ''
        x -= expansion
        y -= expansion
        w += 2*expansion
        h += 2*expansion
        
        if x < 0: 
            x = 0
        if y < 0: 
            y = 0
        if x + w > 640: 
            w = 640 - x
        if y + h > 480:
            h = 480 - y

        face = [x,y,w,h]
        face = [int(el) for el in face]  # integer-ize
        return face

        
if __name__ == "__main__":

    rospy.init_node('filter_face_live')

    face_detector = FaceDetector('/home/ruebenm/workspaces/privacy_ws/src/probability_filters/bags/asus_xtion_camera_info.bag',
                                 '/camera/rgb/image_color')

    rospy.spin()

    """
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
    """
    cv2.destroyAllWindows()
