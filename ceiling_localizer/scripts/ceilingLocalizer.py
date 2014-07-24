#!/usr/bin/env python

import rospy
import cv, cv2
import numpy
import math
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import *
from image_geometry import PinholeCameraModel
from collections import deque


from cv_bridge import CvBridge

# This script is used to localize the robot relative to an overhead camera and then
# publish a frame to tf that is the parent of base_footprint.  There must be a 
# transformation from the /map frame to the optical frame of the overhead camera for
# this method to be used for actual localization.  At present the front of the robot
# is marked by the largest red section in the image and the rear of the robot is
# marked by the largest blue section in the image.  From this the midpoint is 
# calculated for position and the vector from blue -> red is used for orientation.


class ceilingLocalizer():
	def __init__(self, image_topic, camera_topic, robot_depth, robot_frame):

		# A subscriber to the image topic
		rospy.Subscriber(image_topic, Image, self.image_callback)

		# To get which frame is the camera frame
		rospy.Subscriber(camera_topic, CameraInfo, self.camera_callback)

		# To make the pixel to vector projection
		self.camModel = PinholeCameraModel()

		# How far is the robot plane from the camera?
		self.robot_depth = robot_depth

		# What is the name of the frame we should publish?
		self.robot_frame = robot_frame

		self.bridge = CvBridge()

		# Publish frames to tf
		self.broadcaster = tf.TransformBroadcaster()

		# Publisher for edited video
		self.video = rospy.Publisher('ceilingMarker', Image)

	def camera_callback(self, cameraInfo):
		self.cameraInfo = cameraInfo
		self.camModel.fromCameraInfo(cameraInfo)
		self.cameraFrame = cameraInfo.header.frame_id

	def image_callback(self, image_in):
		image_cv = self.bridge.imgmsg_to_cv(image_in, 'bgr8')
		image_cv2 = numpy.array(image_cv, dtype=numpy.uint8)
		#image_cv2 = cv2.GaussianBlur(image_cv2,(5,5), 100, 100)

		# Convert the image from BGR to HSV
		image_hsv_cv2 = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2HSV)

		# Ranges in rgb
		lower_red_rgb = numpy.array([0,0,150])
		upper_red_rgb = numpy.array([150,150,255])
		lower_blue_rgb = numpy.array([150,0,0])
		upper_blue_rgb = numpy.array([255,150,150])
		# Ranges in hsv
		lower_red_hsv = numpy.array([0,50,50])
		upper_red_hsv = numpy.array([10,255,255])
		lower_blue_hsv = numpy.array([105,50,50])
		upper_blue_hsv = numpy.array([120,255,255])
		lower_red_hsv2 = numpy.array([160,50,50])
		upper_red_hsv2 = numpy.array([179,255,255])
		# Make masks from our ranges
		mask_red_rgb = cv2.inRange(image_cv2, lower_red_rgb, upper_red_rgb)
		mask_red_hsv = numpy.bitwise_or(cv2.inRange(image_hsv_cv2, lower_red_hsv, upper_red_hsv), cv2.inRange(image_hsv_cv2, lower_red_hsv2, upper_red_hsv2))
		mask_blue_rgb = cv2.inRange(image_cv2, lower_blue_rgb, upper_blue_rgb)
		mask_blue_hsv = cv2.inRange(image_hsv_cv2, lower_blue_hsv, upper_blue_hsv)
		# Combine masks
		mask_red = numpy.bitwise_and(mask_red_rgb, mask_red_hsv)
		mask_blue = numpy.bitwise_and(mask_blue_rgb, mask_blue_hsv)

		# Copies of masks for imshow
		mask_red_out = numpy.copy(mask_red)
		mask_blue_out = numpy.copy(mask_blue)

		# Find the largest contour in each mask
		front_marker = self.find_marker(mask_red)
		back_marker = self.find_marker(mask_blue)

		# Find the centerpoints
		[frontx,fronty] = self.find_center(front_marker)
		[backx,backy] = self.find_center(back_marker)

		# Find the midpoint
		midx = (frontx+backx)/2
		midy = (fronty+backy)/2
		
		#cv2.circle(image_cv2,(midx,midy), 10, (0,255,0),cv2.cv.CV_FILLED,8,0)

		[vx,vy,vz] = self.camModel.projectPixelTo3dRay((midx,midy))
		marker_depth = self.robot_depth / vz
		
		# Position of the robot relative to ceiling kinect
		x = marker_depth * vx
		y = marker_depth * vy
		z = self.robot_depth

		# Orientation of robot
		yaw = math.atan2((fronty-backy), (frontx-backx))
		quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,yaw, axes='sxyz')

		# Broadcast the frame to tf
		self.broadcaster.sendTransform((x,y,z), quaternion, rospy.Time.now(), self.robot_frame, self.cameraFrame)

		# Draw a line for debug
		self.draw_orientation(image_cv2, (frontx, fronty), (backx, backy))

		# Show all the images
		cv2.imshow('mask_red', mask_red_out)
		cv2.imshow('mask_blue', mask_blue_out)
		cv2.imshow('image', image_cv2)
		cv2.waitKey(3)

		# Convert back to ROS Image msg
		image_cv = cv.fromarray(image_cv2)
		image_out = self.bridge.cv_to_imgmsg(image_cv, 'bgr8') 
		self.video.publish(image_out)

	def draw_orientation(self, image, front, back):
		# Draw a line across the whole image for debug
		slope = float(front[1] - back[1]) / float(front[0] - back[0])
		leftmosty = int((-front[0]*slope) + front[1])
		rightmosty = int(((image.shape[1]-front[0])*slope)+front[1])
		cv2.line(image, (image.shape[1]-1, rightmosty), (0, leftmosty), (0,255,0), 5)

	def find_marker(self, mask):
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		largest = sorted(contours, key = cv2.contourArea,reverse = True)[:1]
		return largest[0]

	def find_center(self, contour):
		moments = cv2.moments(contour, True)
		x = int(moments['m10']/moments['m00'])
		y = int(moments['m01']/moments['m00'])
		return x,y

if __name__ == '__main__':

	rospy.init_node('ceilingLocalizer', log_level=rospy.DEBUG)
	image_topic = rospy.get_param('ceilingLocalizer/image_topic', "/ceiling/rgb/image_color")
	camera_topic = rospy.get_param('ceilingLocalizer/camera_topic', "/ceiling/rgb/camera_info")
	robot_depth = rospy.get_param('ceilingLocalizer/robot_depth', 2.289175)
	robot_frame = rospy.get_param('ceilingLocalizer/robot_frame', "base_top")

	cf = ceilingLocalizer(image_topic, camera_topic, robot_depth, robot_frame)

	rospy.spin()
