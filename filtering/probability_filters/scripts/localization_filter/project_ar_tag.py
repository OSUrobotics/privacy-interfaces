#!/usr/bin/env python

import rospy
import os
import tf
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PolygonStamped
from image_geometry import PinholeCameraModel
import pickle  # gonna need a pickle


class ARTagCornerGrabber():
    def __init__(self):
        self.file = os.environ['HOME'] + '/ar_tag_corners_' + str(rospy.Time.now().to_nsec()) + '.pickle'
        self.lis = tf.TransformListener()
        self.bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.need_info = True
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.info_callback)
        self.need_bounds = True
        rospy.Subscriber('/object_bounds', PolygonStamped, self.bounds_callback)

        while self.need_info and self.need_bounds:
            rospy.sleep(0.01)  # block until have CameraInfo and object bounds

        self.corners = []
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback, queue_size=1)

        #while not rospy.is_shutdown():
        #    self.get_tag_frame()

    def image_callback(self, image_msg):
        print 'Causing an image to be!'
        image = self.bridge.imgmsg_to_cv2(image_msg)

        self.get_tag_frame(image_msg.header.stamp)

        if len(self.corners) > 0:
            for corner in self.corners:
                cv2.circle(image, corner, 3, (255,0,255), -1)
            self.corners = []  # clear the corners
                
        cv2.imshow('Image with AR tag corners shown', image)
        cv2.waitKey(1)

    def info_callback(self, info):
        if self.need_info:
            self.model.fromCameraInfo(info)
            self.need_info = False
            rospy.loginfo('AR_TAG_PROJECTOR> Got CameraInfo!')

    def bounds_callback(self, bounds):
        if self.need_bounds:
            self.bounds = bounds
            self.need_bounds = False
            rospy.loginfo('AR_TAG_PROJECTOR> Got object bounds!')

    def get_tag_frame(self, time):
        print 'Updating tf frame!'
        corners = []
        source_frame = self.bounds.header.frame_id
        target_frame = '/camera_rgb_optical_frame'
        if True: #self.lis.frameExists(source_frame):
            try:
                self.lis.waitForTransform(target_frame, source_frame, time, rospy.Duration(3.0))
                for point in self.bounds.polygon.points:
                    ps = PointStamped()
                    ps.header.stamp = time
                    ps.header.frame_id = source_frame
                    ps.point.x = point.x
                    ps.point.y = point.y
                    ps.point.z = 0.0
                    corner_cam_frame = self.lis.transformPoint(target_frame, ps)
                    uv = self.model.project3dToPixel((corner_cam_frame.point.x,
                                                      corner_cam_frame.point.y,
                                                      corner_cam_frame.point.z))
                    corners.append(tuple(int(el) for el in uv))
                self.corners = corners                
                print corners
                with open(self.file, 'a') as f:
                    pickle.dump((time.to_time(), corners), f)
            except tf.Exception:
                rospy.loginfo('Could not do coordinate transformation.')
        else:
            rospy.loginfo('Frame {0} does not exist.'.format(source_frame))
                

if __name__ == "__main__":

    rospy.init_node('project_ar_tag')
    grabber = ARTagCornerGrabber()
    rospy.spin()

