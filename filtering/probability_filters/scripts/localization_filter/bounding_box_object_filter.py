#!/usr/bin/env python

import rospy
import os
import tf
import cv2
import numpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
import pickle  # gonna need a pickle


class BoundingBoxFilter():
    def __init__(self, only_record):

        print only_record
        self.only_record = only_record
        self.file = os.environ['HOME'] + '/filter_corners_' + str(rospy.Time.now().to_nsec()) + '.pickle'

        self.lis = tf.TransformListener()

        self.need_info = True
        self.model = PinholeCameraModel()

        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.info_callback)

        while self.need_info:
            rospy.sleep(0.01)

        rospy.loginfo('Got CameraInfo! Proceeding to filter.')

        self.have_projections = False
        rospy.Subscriber('/object_vertex_projections', PointCloud, self.projections_callback)

        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)#, queue_size=1)
        self.image_pub = rospy.Publisher('/image_out', Image)

    def info_callback(self, info):
        if self.need_info:
            self.model.fromCameraInfo(info)
            self.need_info = False

    def projections_callback(self, projections):
        self.projections = projections
        if not self.have_projections:
            rospy.loginfo('Got projections! Can now begin filtering.')
            self.have_projections = True

    def image_callback(self, image):
        if self.have_projections:
            self.projections.header.stamp = image.header.stamp
            try:
                #self.lis.waitForTransform('/camera_rgb_optical_frame', self.projections.header.frame_id, image.header.stamp, rospy.Duration(1.0))
                projections = self.lis.transformPointCloud('/camera_rgb_optical_frame', self.projections)  # transform to camera frame
            except Exception: 
                rospy.loginfo('Skipping image!')
                return  # skip this image; the filter probably hasn't caught up yet
            corners = []
            for point in projections.points:  # project rays onto camera image plane
                u, v = self.model.project3dToPixel((point.x,
                                                    point.y,
                                                    point.z))
                corners.append((int(u), int(v)))

            if self.only_record:
                with open(self.file, 'a') as f:
                    pickle.dump((rospy.Time.now().to_time(), corners), f)
                    rospy.loginfo('RECORDED FILTER POINTS!')

            else:
                array = self.bridge.imgmsg_to_cv2(image, "bgr8")
                for i in range(0, len(corners), 4):
                    cv2.rectangle(array, corners[i], corners[i+2],  (0,0,255), 3)
                #corners_convex = cv2.convexHull(numpy.array(corners))  # convex hull algorithm
                #cv2.fillConvexPoly(array, corners_convex, (0, 0, 255))  # fill convex hull
                #for [u, v] in corners:
                #    cv2.circle(array, (int(u), int(v)), 3, (255, 0, 0))  # draw circles at vertices for debugging
                image_new = self.bridge.cv2_to_imgmsg(array, "bgr8")
                image_new.header.stamp = rospy.Time.now()
                self.image_pub.publish(image_new)
    

if __name__ == '__main__':

    rospy.init_node('bounding_filter')
    only_record = rospy.get_param('~only_record', False)
    boundingBoxFilter = BoundingBoxFilter(only_record)
    rospy.spin()
