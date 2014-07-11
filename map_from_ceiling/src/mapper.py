#!/usr/bin/env python
import roslib
import rospy
from visualization_msgs.msg import Marker 
from itertools import combinations
from math import atan, fabs

def is_within(a, b, tolerance):
	return (a + tolerance > b) and (a - tolerance < b)

def print_point(point):
	print "	(" , point.x, "," , point.y, ",", point.z, ")"

class ZMode():
	def __init__(self, z):
		self.z = z

	def add_z(self, z):
		self.z = (self.z + z) / 2.0
		self.number += 1

class LineSegmentTree():
	def __init__(self, point_a, point_b):
		self.a = point_a
		self.b = point_b
		self.normals = []
		self.parent = None

	# Returns the beginning of the tree
	def find_start(self):
		if self.parent = None:
			return self
		else
			return self.parent.find_start()

	# Returns true if the point is either of this's points
	def is_endpoint(self, point):
		return (point == self.find_start().a) or (point == self.find_start().b)

	# Finds all points such that the line defined by point and self.a 
	# (or self.b) are orthogonal to the line defined by self.a and self.b
	def find_orthogonal_points(self, points):
		orthogonal_points = []
		for point in points:
			if self.orthogonal_to(point) and not self.contains(point):
				orthogonal_points.append(point)

	# Returns True if this chain contains the point
	def contains(self, point):
		if self.a == point or self.b == point:
			return True
		elif self.parent == None:
			return False
		else:
			return self.parent.contains(point)

	# Finds all closed chains where this is the starting line segment
	def find_closed_chains(self, points):
		chains = []
		for point in find_orthogonal_points(points):
			if is_endpoint(point):
				chains.append(self.make_chain())
			else:
				chains.append(point.find_closed_chains)
		return chains

	# Returns a list of lineSegmentTrees which is a closed chain
	def make_chain(self):
		if self.parent == None:
			return [self]
		else:
			return self.parent.make_chain().append(self)
	
	# Returns true if the line defined by point and self.a 
	# (or self.b) are orthogonal to the line defined by self.a and self.b
	def orthogonal_to(self, point):
		angle_tolerance = 0.2
		m_AB = (self.a.y - self.b.y)/(self.a.x - self.b.x)
		m_AC = (self.a.y - point.y)/(self.a.x - point.x)
		angle = fabs(atan((m_AB - m_AC)/(1 + m_AB * m_AC)))
		if (is_within(angle, 0, angle_tolerance) or is_within(angle, pi, angle_tolerance) or is_within(angle, pi/2.0, angle_tolerance)):
			return True
		m_BC = (self.b.y - point.y)/(self.b.x - point.x)
		angle = fabs(atan((m_AB - m_BC)/(1 + m_AB * m_BC)))
		if (is_within(angle, 0, angle_tolerance) or is_within(angle, pi, angle_tolerance) or is_within(angle, pi/2.0, angle_tolerance)):
			return True
		return False

if name == '__main__':
	print "Initializing ROS"
	rospy.init_node("mapper")
	print "Waiting for Message"
	msg = rospy.wait_for_message("points", Marker)
	print "Recived Message"
	zmode = []
	print "Removing weak points"
	for i in xrange(0, msg.points.size()):
		# Remove weak points (n < 10) from list
		if msg.colors[i].g < 1:
			print "	Point ", i, " removed"
			msg.points.pop(i)
		# Sort the points into bins by z-value
		for z in zmode:
			if is_within(msg.points[i].z, z, 0.2):
				print "	Added Point ", i, " to bin at z = ", msg.points[i].z
				z.add(msg.points[i].z)
		else:
			print "	Creating bin for Point ", i, " with z = ", msg.points[i].z
			zmode.append(ZMode(msg.points[i].z))
	# find the mode of the z-values
	best_z = 0
	numb_z = 0
	print "Finding the most likely ceiling"
	for z in zmode:
		if z.number > numb_z:
			print "	Better ceiling found with z = ", z.z. , " amd ", z.number, " points"
			numb_z = z.number
			best_z = z.z
	# Removes the points not on the ceiling
	print "Removing points that are not in the ceiling"
	for point in msg.points:
		if not is_within(points.z, best_z, 0.2):
			msg.points.remove(point)
	print len(msg.points), " Points remaining"
	# What remains are good canidates for real corners
	# Find all the closed chains
	print "Finding all closed chains"
	chains = []
	for point_pair in combinations(msg.points, 2):
		print "Finding all closed chains between points:"
		print_point(point_pair[0])
		print_point(point_pair[1])
		root = LineSegmentTree(point_pair[0], point_pair[1])
		chains.append(root.find_closed_chains())
