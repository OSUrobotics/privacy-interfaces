#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32
import csv 
import cv, cv2
import numpy
import tf
from scipy.spatial import ConvexHull

from rosCV import rosCV as rcv

class turtleViz():
	def __init__(self, topic, frames, polygons):
		# The frames and shapes we want to blur
		self.frameLocations = frames
		self.polygonLocations = polygons

		# To make the projection to a pixel
		self.camModel = PinholeCameraModel()

		# To look up the transforms between our points and the camera
		self.listener = tf.TransformListener()
	
		# Alex's image stuff?
		self.rcv = rcv()

		# Our program is driven by this callback
		rospy.Subscriber(topic, Image, self.image_callback)

		# To get which frame is the camera frame
		rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_callback)
		
		# A topic for the edited image data
		self.pub = rospy.Publisher('turtleVision', Image)

	def camera_callback(self, cameraInfo):
		self.cameraInfo = cameraInfo
		self.camModel.fromCameraInfo(cameraInfo)

	def image_callback(self, image_in):
		image_cv2 = self.rcv.toCv2(image_in)
		framesList = []
		with open(self.frameLocations, 'rb') as csvfile:
			reader = csv.reader(csvfile, delimiter=',', quotechar="'")
			for row in reader:
				# Make a list of the frames
				framesList.append(row[0])

		for frame in framesList:
			offset = self.polygonLocations + frame + ".csv"
			polyPoints = []

			# For visualization of polygons in rviz
			polygon = PolygonStamped()
			polygonPub = rospy.Publisher('polygons/' + frame, PolygonStamped)

			# Polygons in rviz are relative to camera frame
			polygon.header.frame_id = "camera_rgb_optical_frame"
			with open(offset, 'rb') as csvfile:
				reader = csv.reader(csvfile, delimiter=',', quotechar="'")
				
				for row in reader:
					newPolyPoint = PointStamped()
					newPolyPoint.header.frame_id = frame
					newPolyPoint.point.x = float(row[0])
					newPolyPoint.point.y = float(row[1])
					newPolyPoint.point.z = float(row[2])
					# We don't care about orientation
					
					try:
						#Transform the point to our published privacy frame
						transformedPolyPoint = self.listener.transformPoint(self.cameraInfo.header.frame_id, newPolyPoint)

						# If the point is behind the camera, don't project it or add it to the polygon
						if not transformedPolyPoint.point.z <= 0:
							# Get the pixel from that transformed point and append it to the list of points
							positionPoint = (transformedPolyPoint.point.x, transformedPolyPoint.point.y, transformedPolyPoint.point.z)
							projectedPoint = self.camModel.project3dToPixel(positionPoint)
							polyPoints.append(projectedPoint)

						# For rviz
						polygon.polygon.points.append(Point32(transformedPolyPoint.point.x, transformedPolyPoint.point.y, transformedPolyPoint.point.z))
					except():
						pass

				#rospy.loginfo(polyPoints)


				#pointArray = numpy.asarray(polyPoints, numpy.int32)
				# Only draw the shape if it has more than 2 points
				if len(polyPoints) > 2:

					hullPoints = self.convexHull(polyPoints)
					pointArray = numpy.asarray(hullPoints, numpy.int32)
					#pointArray2 = numpy.asarray(polyPoints, numpy.int32)

					rospy.loginfo(pointArray)
					#rospy.loginfo(pointArray2)
					
					image_cv2 = self.rcv.redactPolygon(image_cv2, pointArray)
		
			# For rviz
			polygonPub.publish(polygon)

		# Convert back to ROS Image msg
		image_out = self.rcv.toRos(image_cv2)
		self.pub.publish(image_out)
		self.rcv.imshow(image_cv2)


	# Get the convex hull of a set of points
	def convexHull(self, points):
		cv = ConvexHull(points)
		hull_indices = cv.vertices
		hull_points = [points[i] for i in hull_indices]
		return hull_points


if __name__ == '__main__':

	rospy.init_node('turtleVision', log_level=rospy.DEBUG)
	frames = rospy.get_param('turtleVision/frameLocations', "/home/local/CORNELL-COLLEGE/lscott15/privacy/src/privacy-interfaces/privacy/config/frames/cornell.csv")
	polygons = rospy.get_param('turtleVision/polygonLocations', "/home/local/CORNELL-COLLEGE/lscott15/privacy/src/privacy-interfaces/privacy/config/polygons/cornell/")
	topic = rospy.get_param('turtleVision/image_topic', "/camera/rgb/image_color_repub")

	tv = turtleViz(topic, frames, polygons)

	rospy.spin()
