#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PoseStamped
import csv 
import cv, cv2
import numpy
import tf


from rosCV import rosCV as rcv
class turtleViz():
	def __init__(self, topic):
		self.camModel = PinholeCameraModel()
		self.listener = tf.TransformListener()
		self.rcv = rcv()
		rospy.Subscriber(topic, Image, self.image_callback)

		rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_callback)
		self.haveCamera = False
		self.pub = rospy.Publisher('turtleVision', Image)

	def camera_callback(self, cameraInfo):
		if not self.haveCamera:
			#we update our camModel with the cameraInfo to work some magic.
			self.cameraInfo = cameraInfo
			self.camModel.fromCameraInfo(cameraInfo)
			self.haveCamera = True
	def turnAround(self):
		command = Twist()
		command.angular.z = 0.5
		now = rospy.Time.now()

		while rospy.Time.now() - now < 2:
			self.pub.publish(command)

	def image_callback(self, image_in):
		frameLocations = "/nfs/attic/smartw/users/reume02/catkin_ws/src/privacy-interfaces/privacy/config/frames/newThing.csv"
		with open(frameLocations, 'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',', quotechar="'")
			for row in reader:
				newMarker = PoseStamped()
				newMarker.header.frame_id = row[1]
				newMarker.header.stamp = rospy.Time(0)
				newMarker.pose.position.x = float(row[2])
				newMarker.pose.position.y = float(row[3])
				newMarker.pose.position.z = float(row[4])
			
				newMarker.pose.orientation.x = 0.0
				newMarker.pose.orientation.y = 0.0
				newMarker.pose.orientation.z = 0.0
				newMarker.pose.orientation.w = 0.0

		# Import and convert
		image_cv2 = self.rcv.toCv2(image_in)
		try:
			now = rospy.Time.now()
			# self.listener.waitForTransform(self.cameraInfo.header.frame_id, newMarker.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
			newMarker.header.stamp = rospy.Time(0)
			newPose = self.listener.transformPose(self.cameraInfo.header.frame_id, newMarker)
			rospy.logdebug("transform succeeded!")

			pos0 = (newPose.pose.position.x, newPose.pose.position.y, newPose.pose.position.z)
			proj0 = self.camModel.project3dToPixel(pos0)
			x0 = int(proj0[0])
			y0 = int(proj0[1])

			pos1 = (newPose.pose.position.x+0.5, newPose.pose.position.y, newPose.pose.position.z)
			proj1 = self.camModel.project3dToPixel(pos1)
			x1 = int(proj1[0])
			y1 = int(proj1[1])

			pos2 = (newPose.pose.position.x+0.25, newPose.pose.position.y+0.2, newPose.pose.position.z + 0.4)
			proj2 = self.camModel.project3dToPixel(pos2)
			x2 = int(proj2[0])
			y2 = int(proj2[1])

			self.poly = numpy.array( [ [x0,y0], [x1,y1], [x2,y2] ] )

			# image_cv2 = self.rcv.redact(image_cv2, (x,y), (x+50,y+50))
			# Convert back to ROS Image msg
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, AttributeError):
		# except ():
			rospy.logdebug("transform failed")
			# pass
		cv2.fillConvexPoly(image_cv2, self.poly, -1)
		image_out = self.rcv.toRos(image_cv2)
		self.pub.publish(image_out)
		# self.rcv.imshow(image_cv2)


if __name__ == '__main__':

	rospy.init_node('turtleVision', log_level=rospy.DEBUG)

	tv = turtleViz('/camera/rgb/image_color_repub')

	rospy.spin()
