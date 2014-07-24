#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from person_filter.msg import SkeletonArray
import cv
from cv_bridge import CvBridge


class SkeletonSketcher():
    def __init__(self, image_topic, skeletons_topic):
        self.got_skeletons = False
        self.skeleton_array = SkeletonArray()
        self.bridge = CvBridge()
        rospy.Subscriber(skeletons_topic, SkeletonArray, self.skeletons_callback)
        rospy.Subscriber(image_topic, Image, self.image_callback)

    def skeletons_callback(self, skeleton_array):
        self.skeleton_array = skeleton_array
        self.got_skeletons = True
        rospy.loginfo('Got some skeletons!')

    def image_callback(self, image):
        """ Draws joints on image as dots. """
        skeleton_array = self.skeleton_array   # shorthand
        if self.got_skeletons:
            image_cv = self.bridge.imgmsg_to_cv(image)
            for s in range(len(skeleton_array.skeletons)):
                for j in range(len(skeleton_array.skeletons[s].joints)):
                    u = int(skeleton_array.skeletons[s].joints[j].uv[0])
                    v = int(skeleton_array.skeletons[s].joints[j].uv[1])
                    x = skeleton_array.skeletons[s].joints[j].xyz[0]
                    y = skeleton_array.skeletons[s].joints[j].xyz[1]
                    z = skeleton_array.skeletons[s].joints[j].xyz[2]
                    cv.Circle(image_cv, (u, v), 4, (0, 0, 255), -1)
                    print x, y, z, '-', u, v

            print ''
            cv.ShowImage('Image with Skeletons', image_cv)
            cv.WaitKey(5)
    


if __name__ == "__main__":
    rospy.init_node('skeletons_onto_image')
    skeleton_sketcher = SkeletonSketcher('/camera/rgb/image_color', 
                                         '/skeletons_uv')
    rospy.spin()
    
