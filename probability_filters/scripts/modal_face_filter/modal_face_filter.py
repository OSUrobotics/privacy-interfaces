#!/usr/bin/env python
# 
# Expands the filter area when the face detector fails.


import rospy
import os
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
import numpy
import cv2
from cv_bridge import CvBridge


class FaceDetector():
    """ Measures face location, compiles important info. """
    def __init__(self, stillness_radius, mode_topic, torso_topic, info_topic, image_topic):

        self.stillness_radius = stillness_radius

        # Subscribe to mode info and torso location
        self.need_mode = True
        rospy.Subscriber(mode_topic, Bool, self.mode_callback)
        self.need_torso = True
        rospy.Subscriber(torso_topic, PointStamped, self.torso_callback, queue_size=1)
        
        # Camera model stuff
        self.model = PinholeCameraModel()
        self.need_camera_info = True
        rospy.Subscriber(info_topic, CameraInfo, self.info_callback, queue_size=1)

        # Ready the face detector
        self.classifier = cv2.CascadeClassifier(os.environ['ROS_ROOT'] + '/../OpenCV/haarcascades/haarcascade_frontalface_alt.xml')

        # Wait for all those messages to begin arriving
        while self.need_mode or self.need_torso or self.need_camera_info:
            rospy.sleep(0.01)

        # Subscribe to images
        self.have_image = False
        self.bridge = CvBridge()
        rospy.Subscriber(image_topic, Image, self.handle_image, queue_size=1)

    def mode_callback(self, mode):
        self.is_standing_still = mode.data
        if self.need_mode:
            rospy.loginfo('Got first mode message!')
            self.need_mode = False

    def torso_callback(self, torso_location):
        self.torso_location = torso_location
        if self.need_torso:
            rospy.loginfo('Got first torso message!')
            self.need_torso = False

    def info_callback(self, info):
        if self.need_camera_info:
            rospy.loginfo('Got camera info message!')
            self.info = info
            self.model.fromCameraInfo(info)
            self.need_camera_info = False

    def handle_image(self, imgmsg):
        image = self.bridge.imgmsg_to_cv2(imgmsg)
        if not self.have_image:
            self.have_image = True
        success = self.filter_face(image)

    def filter_face(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.classifier.detectMultiScale(gray, 2, 1)
        
        # Mark torso location
        torso = [self.torso_location.point.x,
                 self.torso_location.point.y,
                 self.torso_location.point.z]
        #torso = [el * -1 for el in torso]  # mirror about the origin
        torso_uv = self.model.project3dToPixel(torso)
        torso_uv = tuple(int(el) for el in torso_uv)
        cv2.circle(image, torso_uv, 5, (0,255,0), -1)

        if len(faces) > 0:  # filter the face
            for face in faces:
                (x,y,w,h) = face
                cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 3)  # plot detected faces
        else:  # filter according to the contingency plan
            x,y,w,h = self.expand_filter(torso)
            cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 5)  # plot expanded filter area

        # Use text to indicate mode -- standing or walking
        if self.is_standing_still:
            text = 'Standing'
        else:
            text = 'Walking'
        cv2.putText(image, text, (50, 480-50), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2)

        cv2.imshow('Filtered Image', image)
        cv2.waitKey(1)

    def expand_filter(self, torso):
        """ If the person is walking, filter the whole screen.
        If the person is standing, filter within a certain distance of their torso. """

        # Max image dimensions
        W = self.info.width
        H = self.info.height

        if self.is_standing_still:
            # Do torso +/- radius in x-direction
            left_xyz = list(torso)  # copy
            left_xyz[0] -= self.stillness_radius
            right_xyz = list(torso)  # copy
            right_xyz[0] += self.stillness_radius

            # Project to image plane
            left_uv = self.model.project3dToPixel(left_xyz)
            right_uv = self.model.project3dToPixel(right_xyz)

            # Filter floor to ceiling, -y to +y
            x = max(left_uv[0], 0)
            y = 0
            w = right_uv[0] - left_uv[0]
            w = min(w, W - x)
            h = H

        else:
            x, y, w, h = [0, 0, W, H]  # filter everything            

        return [int(el) for el in [x,y,w,h]]


        
if __name__ == "__main__":

    rospy.init_node('modal_face_filter')
    
    stillness_radius = rospy.get_param('stillness_radius', 0.5)

    face_detector = FaceDetector(stillness_radius,
                                 '/is_standing_still',
                                 '/torso_location',
                                 '/openni/rgb/camera_info',
                                 '/openni/rgb/image_color')

    rospy.spin()

    cv2.destroyAllWindows()
