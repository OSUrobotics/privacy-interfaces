#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseArray
import numpy
import tf
from tf.transformations import *


# Subscribes to robot pose information, either from an array of
#  possible poses or from an estimated pose with covariance
#  matrix. Publishes pose ranges for a specified confidence level.


class AmclParser():

    def __init__(self, use_particles=False, confidence_in_sigmas=2):

        self.conf = confidence_in_sigmas  # confidence level

        self.lis = tf.TransformListener()

        self.pub = rospy.Publisher('/robot_pose_ranges', Float32MultiArray)

        if use_particles:
            rospy.loginfo('Subscribing to cloud of AMCL poses...')
            rospy.logwarn('Ignoring any specified confidence levels!')
            rospy.Subscriber('/particlecloud', PoseArray, self.particle_callback)
        else:
            rospy.loginfo('Subscribing to AMCL-estimated pose with covariance.')
            rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.covariance_callback)


    def particle_callback(self, poses):
        
        # Aggregate pose data by degree of freedom (x, y, rotation about z)
        x = []; y = []; w = []
        for pose in poses.poses:
            q = [pose.orientation.x,
                 pose.orientation.y,
                 pose.orientation.z,
                 pose.orientation.w]
            x_rotation, y_rotation, z_rotation = euler_from_quaternion(q)
            x.append(pose.position.x)
            y.append(pose.position.y)
            w.append(z_rotation)
        
        # Find min & max of each degree of freedom
        x_min = min(x); x_max = max(x)
        y_min = min(y); y_max = max(y)
        w_min = min(w); w_max = max(w)
        
        self.publish_float32multiarray(x_min, x_max, y_min, y_max, w_min, w_max)  # publish!


    def covariance_callback(self, pose):

        # Grab variances and convert to standard deviations
        cov = numpy.asarray(pose.pose.covariance)
        cov.resize((6, 6))
        var = numpy.diag(cov)
        sdev = numpy.sqrt(var)

        # Enforce confidence level
        x_min = pose.pose.pose.position.x - self.conf * sdev[0]
        x_max = pose.pose.pose.position.x + self.conf * sdev[0]
        y_min = pose.pose.pose.position.y - self.conf * sdev[1]
        y_max = pose.pose.pose.position.y + self.conf * sdev[1]
        q = [pose.pose.pose.orientation.x,
             pose.pose.pose.orientation.y,
             pose.pose.pose.orientation.z,
             pose.pose.pose.orientation.w]
        x_rotation, y_rotation, z_rotation = euler_from_quaternion(q)
        w_min = z_rotation - self.conf * sdev[5]
        w_max = z_rotation + self.conf * sdev[5]

        self.publish_float32multiarray(x_min, x_max, y_min, y_max, w_min, w_max)  # publish!


    def publish_float32multiarray(self, x_min, x_max, y_min, y_max, w_min, w_max):

        # Fill a Float32MultiArray with ranges, then publish
        ranges = Float32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = 'dimension'
        dimension.size = 3
        dimension.stride = 2*3
        ranges.layout.dim.append(dimension)
        min_or_max = MultiArrayDimension()
        min_or_max.label = 'minmax'
        min_or_max.size = 2
        min_or_max.stride = 2
        ranges.layout.dim.append(min_or_max)
        ranges.layout.data_offset = 0
        ranges.data = [x_min, x_max, y_min, y_max, w_min, w_max]
        self.pub.publish(ranges)


if __name__ == "__main__":

    rospy.init_node('amcl_parser')

    # Get params
    use_particles = rospy.get_param('~use_particles', False)
    confidence_in_sigmas = rospy.get_param('~confidence_in_sigmas', 2)

    # Run parser
    parser = AmclParser(use_particles, confidence_in_sigmas)
    rospy.spin()
    
