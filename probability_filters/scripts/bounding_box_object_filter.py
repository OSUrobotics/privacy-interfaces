#!/usr/bin/env python

import rospy
import tf
import cv2
import numpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge


class BoundingBoxFilter():
    def __init__(self):

        self.need_info = True
        self.model = PinholeCameraModel()

        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.info_callback)

        while self.need_info:
            rospy.sleep(0.01)

        rospy.loginfo('Got CameraInfo! Proceeding to filter.')

        self.have_projections = False
        rospy.Subscriber('/extreme_vertex_projections', PointCloud, self.projections_callback)
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback)
        
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
            array = self.bridge.imgmsg_to_cv2(image, "bgr8")
            uv_list = []
            for point in self.projections.points:  # project rays onto camera image plane
                u, v = self.model.project3dToPixel((point.x,
                                                  point.y,
                                                  point.z))
                uv_list.append([int(u), int(v)])
            uv_convex = cv2.convexHull(numpy.array(uv_list))  # convex hull algorithm
            cv2.fillConvexPoly(array, uv_convex, (0, 0, 255))  # fill convex hull
            for [u, v] in uv_list:
                cv2.circle(array, (int(u), int(v)), 3, (255, 0, 0))  # draw circles at vertices for debugging
            image_new = self.bridge.cv2_to_imgmsg(array, "bgr8")
            image_new.header.stamp = rospy.Time.now()
            self.image_pub.publish(image_new)
    

if __name__ == '__main__':

    rospy.init_node('bounding_filter')
    boundingBoxFilter = BoundingBoxFilter()
    rospy.spin()
