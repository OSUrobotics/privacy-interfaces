#!/usr/bin/env python

import rospy
import cv, cv2
import numpy
import math
import tf
import scipy.spatial
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import *
from image_geometry import PinholeCameraModel
from collections import deque


from cv_bridge import CvBridge

# This script is used to localize the robot relative to an overhead camera and then
# publish a frame to tf that is the parent of base_footprint.  There must be a 
# transformation from the /map frame to the optical frame of the overhead camera for
# this method to be used for actual localization.  We first find the areas of our image
# that have a certain brightness and find contours from that binary image.  The contours
# are matched against a pattern and front from back is detected using color.


class ceilingLocalizer():
	def __init__(self, image_topic, camera_topic, robot_depth, robot_frame, pattern):

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

		# Pattern to match
		self.image_orig = cv2.imread(pattern, 1)
		image_bin = cv2.cvtColor(self.image_orig, cv2.COLOR_BGR2GRAY)
		ret, image_bin = cv2.threshold(image_bin, 127,255,cv2.THRESH_BINARY_INV)
		contours, hierarchy = cv2.findContours(image_bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		sorted_contours = sorted(contours, key = cv2.contourArea, reverse = True)
		# Our pattern is the second contour because for some reason its finding the edges of our image as a contour
		self.pattern = sorted_contours[1]
		self.pattern_area = 200

		# To convert images from ROS to CV formats
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
		# Convert the image to grayscale
		image_gray = cv2.cvtColor(image_cv2, cv2.COLOR_BGR2GRAY)
		# Threshold grayscale image
		flag, thresh = cv2.threshold(image_gray, 230, 255, cv2.THRESH_BINARY)

		# Make a copy for contour detection
		thresh_copy = numpy.copy(thresh)

		contours, hierarchy = cv2.findContours(thresh_copy, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		# Sort contours by how much they match our pattern
		potential_markers = self.find_potential_markers(contours)

		# Ranges in hsv
		lower_red_hsv = numpy.array([0,50,50])
		upper_red_hsv = numpy.array([10,255,255])
		lower_blue_hsv = numpy.array([105,50,50])
		upper_blue_hsv = numpy.array([120,255,255])
		lower_red_hsv2 = numpy.array([160,50,50])
		upper_red_hsv2 = numpy.array([179,255,255])
		# Make masks from our ranges
		mask_red_hsv = numpy.bitwise_or(cv2.inRange(image_hsv_cv2, lower_red_hsv, upper_red_hsv), cv2.inRange(image_hsv_cv2, lower_red_hsv2, upper_red_hsv2))
		mask_blue_hsv = cv2.inRange(image_hsv_cv2, lower_blue_hsv, upper_blue_hsv)

		# Front should be red, rear should be blue
		front_mask = mask_red_hsv
		rear_mask = mask_blue_hsv


		
		if len(potential_markers) >= 2:
			# Set one marker as the front, the other the rear
			if self.isFrontDominant(potential_markers[0], front_mask, rear_mask):
				front_marker = potential_markers[0]
				rear_marker = potential_markers[1]
			else
				front_marker = potential_markers[1]
				rear_marker = potential_markers[0]


			# Find the centerpoints
			[frontx,fronty] = self.find_center(front_marker)
			[backx,backy] = self.find_center(rear_marker)
			# Find the midpoint
			midx = (frontx+backx)/2
			midy = (fronty+backy)/2

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

			self.draw_orientation(image_cv2, (frontx, fronty), (rearx, reary))

		else:
			rospy.loginfo("No marker found")



		cv2.imshow('image', image_cv2)
		cv2.imshow('image_gray', image_gray)
		cv2.imshow('pattern', self.image_orig)
		cv2.imshow('image_corners', image_corners)
		cv2.imshow('thresh', thresh)
		cv2.waitKey(3)

		# Convert back to ROS Image msg
		image_cv = cv.fromarray(image_cv2)
		image_out = self.bridge.cv_to_imgmsg(image_cv, 'bgr8') 
		self.video.publish(image_out)

	def isFrontDominant(self, contour, front_mask, rear_mask):
		# Is the contour more red or more blue?
		x,y,w,h = cv2.boundingRect(contour)
		x1 = x - w/2
		y1 = y - h/2
		x2 = x + w/2
		y2 = y + h/2

		cropped_front = front_mask[y1:y2, x1:x2]
		cropped_rear = rear_mask[y1:y2, x1:x2]
		front_ratio = numpy.count_nonzero(cropped_front)/cropped_front.size
		rear_ratio = numpy.count_nonzero(cropped_rear)/cropped_rear.size

		if front_ratio >= rear_ratio:
			return True
		else
			return False

	def find_potential_markers(self, contours):
		# Get rid of small contours and order by match strength
		potential_markers = []
		for contour in contours:
			if cv2.contourArea(contour) > self.pattern_area:
				potential_markers.append(contour)
		sorted_markers = sorted(potential_markers, key=self.match_pattern)
		return sorted_markers

	def match_pattern(self, contour):
		# Match the contour against our pattern
		match = cv2.matchShapes(contour,self.pattern, 1, 0.0)
		return match

	def draw_orientation(self, image, front, back):
		# Draw a line across the whole image for debug
		slope = float(front[1] - back[1]) / float(front[0] - back[0])
		leftmosty = int((-front[0]*slope) + front[1])
		rightmosty = int(((image.shape[1]-front[0])*slope)+front[1])
		cv2.line(image, (image.shape[1]-1, rightmosty), (0, leftmosty), (0,255,0), 3)

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
	pattern = rospy.get_param('ceilingLocalizer/pattern', "/home/local/CORNELL-COLLEGE/lscott15/privacy/src/privacy-interfaces/ceiling_localizer/checkerboard2.png")

	cf = ceilingLocalizer(image_topic, camera_topic, robot_depth, robot_frame, pattern)

	rospy.spin()
