#!/usr/bin/env python

import rospy
import os
import tf
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from image_geometry import PinholeCameraModel
from ar_track_alvar.msg import AlvarMarkers
import pickle  # gonna need a pickle


class ARTagCornerGrabber():
    def __init__(self):
        self.file = os.environ['HOME'] + '/ar_tag_corners_' + str(rospy.Time.now().to_nsec()) + '.pickle'
        self.lis = tf.TransformListener()
        self.bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.need_info = True
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.info_callback)
        self.marker = ()
        #rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.markers_callback)

        while self.need_info:
            rospy.sleep(0.01)  # block until have CameraInfo

        self.corners = []
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback, queue_size=1)

        #while not rospy.is_shutdown():
        #    self.get_tag_frame()

    def image_callback(self, image_msg):
        print 'Causing an image to be!'
        image = self.bridge.imgmsg_to_cv2(image_msg)

        #if len(self.marker) > 0:
        #    cv2.circle(image, self.marker, 3, (0,255,0), -1)

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

    def markers_callback(self, markers):
        if len(markers.markers) > 0:
            for marker in markers.markers:
                if marker.id == 203:
                    #rospy.loginfo('Found marker #203!')
                    uv = self.model.project3dToPixel((marker.pose.pose.position.x,
                                                      marker.pose.pose.position.y,
                                                      marker.pose.pose.position.z))
                    self.marker = tuple(int(el) for el in uv)

    def get_tag_frame(self, time):
        print 'Updating tf frame!'
        corners = []
        d = 0.048  # half a side length
        frame_id = 'ar_marker_203'
        target_frame = '/camera_rgb_optical_frame'
        if self.lis.frameExists(frame_id):
            corner = PointStamped()
            corner.header.frame_id = frame_id
            corner.header.stamp = time
            try:
                self.lis.waitForTransform(target_frame, frame_id, time, rospy.Duration(3.0))
                for dx, dy in zip([d, d, -d, -d], [-d, d, -d, d]):
                    corner.point.x = dx
                    corner.point.y = dy
                    corner.point.z = 0.0
                    corner_cam_frame = self.lis.transformPoint(target_frame, corner)
                    uv = self.model.project3dToPixel((corner_cam_frame.point.x,
                                                      corner_cam_frame.point.y,
                                                      corner_cam_frame.point.z))
                    corners.append(tuple(int(el) for el in uv))
                self.corners = corners                
                with open(self.file, 'a') as f:
                    pickle.dump((time.to_time(), corners), f)
            except tf.Exception:
                rospy.loginfo('Could not do coordinate transformation.')
        else:
            rospy.loginfo('AR Tag frame does not exist.')
                

if __name__ == "__main__":

    rospy.init_node('project_ar_tag')
    grabber = ARTagCornerGrabber()
    rospy.spin()

