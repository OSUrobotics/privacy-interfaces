#!/usr/bin/env python
# 

import rospy
import os
import sys
import rosbag
from sensor_msgs.msg import Image, CameraInfo
import numpy
import cv2
from cv_bridge import CvBridge


class ImageHandler():
    """ Holds Image info; steps through frames """
    def __init__(self):
        self.image = None
        self.bridge = CvBridge()
        self.first_frame = True

    def open_bag(self, bag_path):
        self.bag = rosbag.Bag(bag_path)
        self.images = self.bag.read_messages(topics='/camera/rgb/image_color')
        
    def get_image(self):
        print 'Incrementing!'

        # Handle time
        imgmsg = self.images.next()[1]
        self.image = self.bridge.imgmsg_to_cv2(imgmsg)

        if self.first_frame:
            self.t = 0.0
            self.t_ref = imgmsg.header.stamp.to_sec()
            self.first_frame = False
        else:
            self.t = imgmsg.header.stamp.to_sec() - self.t_ref
        

class FaceDetector():
    """ Determines face location and infers velocity. """
    def __init__(self):
        self.classifier = cv2.CascadeClassifier(os.environ['ROS_ROOT'] + '/../OpenCV/haarcascades/haarcascade_frontalface_alt.xml')
        self.faces = numpy.ma.asarray([[0, 0, 0, 0, 0, 0, 0, 0, 0], 
                                       [0, 0, 0, 0, 0, 0, 0, 0, 0]])  # two (2) dummy measurements
        self.faces[0:2] = numpy.ma.masked

    def detect_face(self, t, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = self.classifier.detectMultiScale(gray, 1.3, 5)
        if len(faces) > 0:
            (x,y,w,h) = faces[0]
            face = numpy.asarray([t, x + w/2, y + h/2, w, h, 0, 0, 0, 0])
            self.faces = numpy.ma.concatenate((self.faces, [face]))
            cv2.rectangle(image, (x,y), (x+w,y+h), (0,255,0), 1)  # plot detected face
        else:
            face = numpy.asarray([t, 0, 0, 0, 0, 0, 0, 0, 0])
            self.faces = numpy.ma.concatenate((self.faces, [face]))
            self.faces[-1] = numpy.ma.masked

        new = self.faces[-1]
        old = self.faces[-2]
        if not any(new.mask) and not any(old.mask):  # if both faces are good
            dt = self.measure_speeds(new, old)
            (du, dv, dw, dh) = new[5:9]
            cv2.putText(image, 'Lateral speed: {0}pix/frame, Embiggening rate: {1}pix/frame'.format(int(du*dt), int(dw*dt)),
                        (20, 480-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        
        cv2.imshow('Image with Face(s)', image)
        cv2.waitKey(10)

        return new

    def measure_speeds(self, new, old):
        """ If previous two faces were good, returns the speed (dPixel/dTime). """
        dt = new[0] - old[0]
        new[5:9] = (new[1:5] - old[1:5]) / dt
        return dt

    """
    def write_face(self):
        image_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
        mask_msg = self.bridge.cv2_to_imgmsg(self.mask, "bgr8")
        self.bag_out.write(self.topic_image, image_msg)
        self.bag_out.write(self.topic_mask, mask_msg)
        rospy.loginfo('Wrote a frame to bag file!')
    """    


def measure_face_speeds():
    """ Main function """
    rospy.init_node('measure_face_speeds')
    
    # Handle arguments
    if len(sys.argv) < 2:
        rospy.logerr('Must pass at least the first argument: the INPUT bag file path.')
        return -1
    else:
        path_bag_in = sys.argv[1] 

    # Initialize object(s)
    image_handler = ImageHandler()
    image_handler.open_bag(path_bag_in)

    face_detector = FaceDetector()

    while not rospy.is_shutdown():
        try:
            image_handler.get_image()
            face = face_detector.detect_face(image_handler.t,
                                             image_handler.image)
            print face
        except StopIteration:
            rospy.loginfo('Processed last image; exiting!')
            return 0


if __name__ == "__main__":

    measure_face_speeds()
    
    cv2.destroyAllWindows()
