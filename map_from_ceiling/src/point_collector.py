#!/usr/bin/env python
import roslib
import rospy
import tf
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker 

def is_within(a, b, tolerance):
	return (a + tolerance > b) and (a - tolerance < b)

def print_point(point):
	print "	(" , point.x, "," , point.y, ",", point.z, ")"

class Point_Cluster():
	def __init__(self, point):
		self.x = point.x
		self.y = point.y
		self.z = point.z
		self.green = 0.1
		self.number = 1

	def add_point(self, point):
		self.x = (point.x + self.x) / 2.0
		self.y = (point.y + self.y) / 2.0
		self.z = (point.z + self.z) / 2.0
		self.number += 1
		if (self.green < 1):
			self.green += 0.1

	def is_close(self, point):
		return is_within(point.x, self.x, 0.5) and is_within(point.y, self.y, 0.5) and is_within(point.z, self.z, 0.5)

	def to_point(self):
		return Point(self.x, self.y, self.z)

	def to_color(self):
		return ColorRGBA(0.0, self.green, 0.0, 1.0)

class Marker_object():
	marker = Marker()
	clusters = []

	def __init__(self):
		self.marker.header.frame_id = "odom"
		self.marker.ns = "points_and_lines"
		self.marker.id = 0
		self.marker.action = Marker.ADD
		self.marker.pose.orientation.w = 1.0
		self.marker.type = Marker.POINTS
		self.marker.lifetime = rospy.Duration(0.5)
		#self.marker.color.g = 1.0
		#self.marker.color.a = 1.0
		self.marker.scale.x = 0.1
		self.marker.scale.y = 0.1
		#self.marker.points = []

	def add_point(self, point):
		for point_cluster in self.clusters:
			if point_cluster.is_close(point):
				print "Found a close point"
				point_cluster.add_point(point)
				break;
		else found_close_point:
			print "No close points"
			self.clusters.append(Point_Cluster(point))
		self.flush_clusters()

		"""
		self.marker.header.stamp = rospy.Time.now()
		if (len(self.marker.points) > 30):
			self.marker.points.pop(0)
		self.marker.points.append(point)
		"""
	
	def flush_clusters(self):
		print "All points found:"
		self.marker.header.stamp = rospy.Time.now()
		self.marker.points = []
		self.marker.colors = []
		for cluster in self.clusters:
			print_point(cluster.to_point())
			print "		Number points: ", cluster.number
			self.marker.points.append(cluster.to_point())
			self.marker.colors.append(cluster.to_color())

marker = Marker_object()

def callback(data):
	global marker
	print "Received Point"
	print "Point: "
	print_point(data.point)
	tf_listener.waitForTransform("camera_rgb_optical_frame", "odom", rospy.Time.now(), rospy.Duration(4.0))
	new_point = tf_listener.transformPoint("odom", data)
	print "New Point"
	print_point(new_point.point)

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