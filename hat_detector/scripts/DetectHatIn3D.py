#! /usr/bin/env python

import rospy
import cv, cv2
import numpy as np
import message_filters
from std_msgs import *
from image_geometry import PinholeCameraModel  
from sensor_msgs.msg import Image, CameraInfo, PointCloud 
from cmvision.msg import Blobs
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped
import code

class DrawRect():
	def __init__(self, topic_image, topic_blob, topic_info, topic_out):
		
		self.sync1_callback = False #synchronize the image
		self.sync2_callback = False #synchronize the camera info
				
		self.bridge = CvBridge()
		rospy.Subscriber(topic_image, Image, self.image_callback)
		rospy.Subscriber(topic_blob, Blobs, self.blob_callback)		
		rospy.Subscriber(topic_info, CameraInfo, self.info_callback)		

		self.pub = rospy.Publisher(topic_out, PointStamped)		

	def image_callback(self, image):
		self.sync1_callback = True
		self.image = self.bridge.imgmsg_to_cv(image)
		self.image = np.asarray(self.image)

	def info_callback (self, info):
		#get the camera information		
		self.sync2_callback = True
		self.info = info
		
	def blob_callback(self, blob):
		if self.sync1_callback and self.sync2_callback and (blob.blob_count != 0):
			bleb = self.FindBiggestBlob (blob)
			z = self.image[bleb.y][bleb.x]
	
			self.UVtoXYZ = PinholeCameraModel()
			self.UVtoXYZ.fromCameraInfo(self.info)
			vec = self.UVtoXYZ.projectPixelTo3dRay ((bleb.x, bleb.y))	
			vec = [x * (z/vec[2]) for x in vec]

			self.P = PointStamped()
			self.P.point.x = vec[0]
			#the np make the downward direction positive direction,
			#so we need to multiply to -1 to make upward direction
			#positive direction			
			self.P.point.y = vec[1]
			self.P.point.z = vec[2]
			
			print self.P			
			self.pub.publish(self.P)						
						
	def FindBiggestBlob(self, blob):
		#Assume the hat is the only green blob in front of the camera		
		x = 0		
		bleb = None
		while x < blob.blob_count:
			if (x == 0):
				bleb = blob.blobs[x]
			else:
				if (blob.blobs[x].area > bleb.area):
					bleb = blob.blobs[x]
			x = x + 1
  		return bleb
			
	
if __name__ == '__main__':

	rospy.init_node('hat_detect')

	filler = DrawRect('/camera/depth_registered/image_raw','/blobs','/camera/rgb/camera_info','/center_of_hat')
	
	rospy.spin()
