#! /usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int8
	

class HattedHeadDetector():
	def __init__(self):
		# Initialize hat stuff
		self.have_hat_center = False
		rospy.Subscriber('/center_of_hat', PointStamped, self.hat_callback)

		# TF Listener
		self.listener = tf.TransformListener()
		rospy.sleep(1.0)  # give listener time to initialize

		# Loop 
		self.pub = rospy.Publisher('/hatted_head', Int8)
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			hatted_head = Int8()
			hatted_head.data = 0
			if self.have_hat_center:

				self.get_head_names()
				hat_wrt_heads = self.hat_wrt_heads()
				is_on_heads = []
				for hat_wrt_head in hat_wrt_heads:
					is_on_head = self.detect_hatted_head(hat_wrt_head)
					is_on_heads.append(is_on_head)
				if sum(is_on_heads) > 1:
					rospy.logerr('More than one head is wearing the hat!')
				elif sum(is_on_heads) == 1:
					which_point = is_on_heads.index(True)
					which_head = int(self.joints[which_point][-1])
					hatted_head.data = which_head
					rospy.loginfo('HAT ON HEAD #{0}!'.format(which_head))
				else:
					rospy.loginfo('Nope, no heads are hatted...')
			self.pub.publish(hatted_head)
			r.sleep()
			
	def hat_callback(self, hat_center):
		""" Get hat center locations. """
		self.hat_center = hat_center
		self.have_hat_center = True

	def get_head_names(self):
		""" Get tf frame_id for each head. """
		frames = self.listener.getFrameStrings()
		self.joints = []
		for n in range(1,10):
			suffix = '_' + str(n)
			joint = 'head' + suffix  # generate joint name
			if joint in frames:
				self.joints.append(joint)
			else:
				# print 'This many skeletons: ' + str(n-1)
				break

	def hat_wrt_heads(self):
		""" Transform hat location to all head frames. """
		hat_wrt_heads = []
		for joint in self.joints:
			hat_wrt_head = self.hat_wrt_frame(joint)
			hat_wrt_heads.append(hat_wrt_head)
		return hat_wrt_heads
			

	def hat_wrt_frame(self, frame):
		""" Transform hat location to given frame. """
		self.listener.waitForTransform(self.hat_center.header.frame_id, frame, rospy.Time(0), rospy.Duration(1.0))
		hat_wrt_frame = self.listener.transformPoint(frame, self.hat_center)
		return hat_wrt_frame

	def detect_hatted_head(self, hat_wrt_head):
		""" Use spatial reasoning to determine whether each head is hatted. """
		is_on_head = hat_wrt_head.point.y > 0.0 and hat_wrt_head.point.y < 0.20 and abs(hat_wrt_head.point.x) < 0.10 and abs(hat_wrt_head.point.z) < 0.10
		return is_on_head
		
	
if __name__ == '__main__':
	rospy.init_node('is_hat_on_head')
	detector = HattedHeadDetector()
	rospy.spin()


# Outline:
# +Subscribe to hat location
# +Get hat location in head frame(s)
#   +Persistently check which head numbers exist
#   +For each extant head, do the transform
# +Check for hat wearing
#   +Throw error if there are multiple wearers
# +Publish an Int8 of who is wearing the hat
