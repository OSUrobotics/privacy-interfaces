#!/usr/bin/env python
# 
# Detects face using hat position, real face size, and depth information.
# Creates mask of face pixels to be used as ground truth.
# 

import rospy
import os
import sys
import rosbag
from sensor_msgs.msg import Image
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
        #camera_info = self.bag.read_messages(topics='/camera/rgb/camera_info').next()[1]
        #self.camera_model = PinholeCameraModel()
        #self.camera_model.fromCameraInfo(camera_info)

    def get_image(self):
        imgmsg = self.images.next()[1]
        self.image = self.bridge.imgmsg_to_cv2(imgmsg)


class FaceLabeller():        
    """ Prompts user to click on image; stores and saves face masks. """ 
    def __init__(self):
        self.faces = []
        self.keep_going = True
        self.bridge = CvBridge()
        
    def open_bag(self, bag_path):
        self.bag = rosbag.Bag(bag_path, 'w')
        self.topic_image = 'image'
        self.topic_mask = 'mask'


    def label_face(self, image):
        self.image = image
        image_temp = cv2.pyrUp(image)
        name = 'Current image -- see terminal window for instructions.'
        msgs = ['Click the top of the forehead!',
                'Click the bottom of the chin!',
                'Click the leftmost part of the head! (usually an ear)',
                'Click the rightmost part of the head! (usually an ear)',
                'How does this look?']

        self.clicked = []
        self.face = [image_temp.shape[1], image_temp.shape[0], -1, -1]
        for msg in msgs:
            cv2.namedWindow(name)
            cv2.rectangle(image_temp, tuple(self.face[0:2]), tuple(self.face[2:4]), (0,0,255), 1)
            cv2.imshow(name, image_temp)
            cv2.setMouseCallback(name, self.mouse_callback)
            self.need_click = True
            rospy.loginfo(msg)
            key = cv2.waitKey(0)
            if key == 1048696:  # the letter "x", lowercase
                self.keep_going = False
                rospy.loginfo('Quitting!')
                break
            else: 
                self.face[0] = min(self.face[0], self.clicked[-1][0])  # left boundary
                self.face[1] = min(self.face[1], self.clicked[-1][1])  # top boundary
                self.face[2] = max(self.face[2], self.clicked[-1][0])  # right boundary
                self.face[3] = max(self.face[3], self.clicked[-1][1])  # bottom boundary

        mask_temp = self.make_mask(image_temp)
        self.mask = cv2.pyrDown(mask_temp)
            
    def mouse_callback(self, event, u, v, foo, bar):
        if event == cv2.EVENT_LBUTTONDOWN and self.need_click:
            self.clicked.append([u, v])
            self.need_click = False
            rospy.loginfo('Got your click! Press "x" to quit or any other key to continue.')

    def make_mask(self, image):
        mask = numpy.zeros(image.shape)
        mask[self.face[1] : self.face[3],
                  self.face[0] : self.face[2], :] = 255
        mask = mask.astype('uint8')
        return mask

    def write_face(self):
        image_msg = self.bridge.cv2_to_imgmsg(self.image, "bgr8")
        mask_msg = self.bridge.cv2_to_imgmsg(self.mask, "bgr8")
        self.bag.write(self.topic_image, image_msg)
        self.bag.write(self.topic_mask, mask_msg)
        rospy.loginfo('Wrote a frame to bag file!')


def label_faces():
    """ Main function """
    rospy.init_node('label_face_manually')
    
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

    # Initialize objects
    image_handler = ImageHandler()
    image_handler.open_bag(path_bag_in)

    face_labeller = FaceLabeller()
    face_labeller.open_bag(path_bag_out)

    while face_labeller.keep_going:
        # Get next image
        try:  
            image_handler.get_image()
        except StopIteration:
            rospy.loginfo('Processed last image; exiting!')
            return 0

        # Label that image
        face_labeller.label_face(image_handler.image)

        face_labeller.write_face()

    image_handler.bag.close()
    face_labeller.bag.close()
        



if __name__ == "__main__":

    label_faces()
    
    cv2.destroyAllWindows()
