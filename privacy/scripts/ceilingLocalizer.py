#!/usr/bin/env python

import rospy
import cv, cv2
import numpy
import math
import tf
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import *
from image_geometry import PinholeCameraModel


from rosCV import rosCV as rcv

# The ranges for different colors
#RED = (((0,50,50),(10,255,255))((160,50,50),(179,255,255)))
#BLUE = ((90,50,50),(120,255,255))


class ceilingLocalizer():
	def __init__(self, image_topic, camera_topic, robot_depth):

		# A subscriber to the image topic
		rospy.Subscriber(image_topic, Image, self.image_callback)

		# To get which frame is the camera frame
		rospy.Subscriber(camera_topic, CameraInfo, self.camera_callback)

		# To make the pixel to vector projection
		self.camModel = PinholeCameraModel()

		# How far is the robot plane from the camera?
		self.robot_depth = robot_depth

		# Alex's image stuff?
		self.rcv = rcv()

		# Publish frames to tf
		self.broadcaster = tf.TransformBroadcaster()

	def camera_callback(self, cameraInfo):
		self.cameraInfo = cameraInfo
		self.camModel.fromCameraInfo(cameraInfo)
		self.cameraFrame = self.camModel.tfFrame()

	def image_callback(self, image_in):
		image_cv2 = self.rcv.toCv2(image_in)
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

		# Find the unit vector
		distance = math.sqrt(math.pow(backx-frontx, 2) + math.pow(backy-fronty, 2))
		uvx = (frontx-backx) / distance
		uvy = (fronty-backy) / distance

		cv2.circle(image_cv2,(midx,midy), 10, (0,255,0),cv2.cv.CV_FILLED,8,0)

		# The adjacent side of our triangle (from kinect to floor minus turtlebot height)
		adjacent = self.robot_depth

		[vx,vy,vz] = self.camModel.projectPixelTo3dRay((midx,midy))
		marker_depth = adjacent / vz
		
		# Position of the robot relative to ceiling kinect
		x = marker_depth * vx
		y = marker_depth * vy
		z = adjacent

		# Orientation of robot
		yaw = math.atan2((fronty-backy), (frontx-backx))
		quaternion = tf.transformations.quaternion_from_euler(0.0,0.0,yaw, axes='sxyz')

		# Broadcast the frame to tf
		self.broadcaster.sendTransform((x,y,z), quaternion, rospy.Time.now(), "base_top", self.cameraFrame)

		# Show all the images
		cv2.imshow('mask_red', mask_red_out)
		cv2.imshow('mask_blue', mask_blue_out)
		cv2.imshow('image', image_cv2)
		cv2.waitKey(3)

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
	camera_topic = rospy.get_param('ceilingLocalizer/camera_topic', "RED")
	robot_depth = rospy.get_param('ceilingLocalizer/robot_depth', 2.289175)

	cf = ceilingLocalizer(image_topic, camera_topic, robot_depth)

	rospy.spin()
