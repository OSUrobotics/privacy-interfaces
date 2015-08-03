#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from person_filter.msg import SkeletonArray
import numpy
import cv
from cv_bridge import CvBridge
import message_filters


class SkeletonSketcher():
    def __init__(self, image_topic, depth_topic, skeletons_topic):

        self.bridge = CvBridge()

        # Synchronized subscribers
        sub_rgb = message_filters.Subscriber(image_topic, Image)
        sub_depth = message_filters.Subscriber(depth_topic, Image)
        sub_skeletons = message_filters.Subscriber(skeletons_topic, SkeletonArray)
        ts = message_filters.TimeSynchronizer([sub_rgb, sub_depth, sub_skeletons], 1)
        ts.registerCallback(self.callback)


    def callback(self, image, depth, skeleton_array):
        # Calculate Z-extents of first skeleton ... VERY INEFFICIENT
        self.z_min = 100
        self.z_max = 0
        for joint in skeleton_array.skeletons[0].joints:
            self.z_min = min(self.z_min, joint.xyz[2])
            self.z_max = max(self.z_max, joint.xyz[2])
        rospy.loginfo('First skeleton has min depth of {0}m and max depth of {1}m.'.format(self.z_min, self.z_max))

        # Make masks from depth image
        # Note: depth values are in millimeters!
        depth_cv = self.bridge.imgmsg_to_cv(depth)  
        depth_cv2 = numpy.asarray(depth_cv)
        self.good_mask = numpy.where((depth_cv2 > self.z_min * 1000 - 0.50) & (depth_cv2 < self.z_max * 1000 + 0.50), True, False)
        self.bad_mask = numpy.logical_not(self.good_mask)

        # Use RGB image
        image_cv = self.bridge.imgmsg_to_cv(image)
        image_cv2 = numpy.asarray(image_cv)
        image_cv2[self.bad_mask] = 0
        image_cv = cv.fromarray(image_cv2)
                
        # Draw joints on image            
        for s in range(len(skeleton_array.skeletons)):
            for j in range(len(skeleton_array.skeletons[s].joints)):
                u = int(skeleton_array.skeletons[s].joints[j].uv[0])
                v = int(skeleton_array.skeletons[s].joints[j].uv[1])
                x = skeleton_array.skeletons[s].joints[j].xyz[0]
                y = skeleton_array.skeletons[s].joints[j].xyz[1]
                z = skeleton_array.skeletons[s].joints[j].xyz[2]
                cv.Circle(image_cv, (u, v), 4, (0, 0, 255), -1)
                #print x, y, z, '-', u, v

        #print ''
        cv.ShowImage('Image with Skeletons', image_cv)
        cv.WaitKey(5)


if __name__ == "__main__":
    rospy.init_node('skeletons_onto_image')
    skeleton_sketcher = SkeletonSketcher('/camera/rgb/image_color/sync', 
                                         '/camera/depth_registered/image_raw/sync',
                                         '/skeletons_uv/sync')
    rospy.spin()
    
