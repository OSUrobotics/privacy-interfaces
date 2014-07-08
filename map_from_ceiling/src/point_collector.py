#!/usr/bin/env python
import roslib
import rospy
import tf
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker 
 
class Marker_object():
	marker = Marker()

	def __init__(self):
		self.marker.header.frame_id = "odom"
		self.marker.ns = "points_and_lines"
		self.marker.id = 0
		self.marker.action = Marker.ADD
		self.marker.pose.orientation.w = 1.0
		self.marker.type = Marker.POINTS
		self.marker.lifetime = rospy.Duration(0.5)
		self.marker.color.g = 1.0
		self.marker.color.a = 1.0
		self.marker.scale.x = 0.1
		self.marker.scale.y = 0.1
		self.marker.points = []

	def add_point(self, point):
		self.marker.header.stamp = rospy.Time.now()
		if (len(self.marker.points) > 30):
			self.marker.points.pop(0)
		self.marker.points.append(point)
		
marker = Marker_object()

def callback(data):
	global marker
	print "Received Point"
	print "Point is (" , data.point.x , ", " , data.point.y , ", " , data.point.z , ")"
	#tf_listener.waitForTransform("camera_rgb_optical_frame", "odom", rospy.Time.now(), rospy.Duration(4.0))
	new_point = tf_listener.transformPoint("odom", data)
	print "New Point is (" , new_point.point.x , ", " , new_point.point.y , ", " , new_point.point.z , ")"
	
	print "Setting up Marker"
	marker.add_point(new_point.point)

if __name__ == '__main__':
	print "Initializing ROS"
	rospy.init_node("point_listener")
	tf_listener = tf.TransformListener()
	rospy.Subscriber("/output", PointStamped, callback)
	pub = rospy.Publisher("/points", Marker, queue_size=10)

	rate = rospy.Rate(10.0)
	print "Ready to Recive Messages"
	while not rospy.is_shutdown():
		pub.publish(marker.marker)