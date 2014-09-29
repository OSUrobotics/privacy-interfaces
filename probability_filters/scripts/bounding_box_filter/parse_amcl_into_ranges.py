#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseArray
import numpy
import tf
from tf.transformations import *
import message_filters
from amcl.msg import ParticleCloudWeights


# Subscribes to robot pose information from an array of possible poses
#  and publishes the most likely poses that add up to a user-specified
#  confidence level.


class AmclParser():

    def __init__(self, confidence=0.50):

        self.confidence = confidence  # confidence level

        self.lis = tf.TransformListener()

        self.cloud_pub = rospy.Publisher('/particlecloud_chosen', PoseArray)
        self.weights_pub = rospy.Publisher('/particlecloud_weights_chosen', ParticleCloudWeights)

        rospy.loginfo('Subscribing to cloud of weighted AMCL poses...')
        cloud_sub = message_filters.Subscriber('/particlecloud', PoseArray)
        weights_sub = message_filters.Subscriber('/particlecloud_weights', ParticleCloudWeights)
        ts = message_filters.TimeSynchronizer([cloud_sub, weights_sub], 10)
        ts.registerCallback(self.particle_callback)


    def particle_callback(self, poses, weights):
        
        # Use Decorate-Sort-Undecorate idiom
        weights_enumerated = [(weight.data, i) for i, weight in enumerate(weights.weights)]
        weights_enumerated.sort(reverse=True)
        for i in range(len(weights_enumerated)):
            if i > 0:
                weights_enumerated[i][0] += weights_enumerated[i-1][0]  # add previous weight
            if weights_enumerated[i][0] > self.confidence:
                rospy.loginfo('Selected {0} poses.'.format(i+1))
                break
        indices = [el[1] for el in weights_enumerated[:i+1]]
        poses.poses = [poses.poses[index] for index in indices]
        weights.weights = [weights.weights[index] for index in indices]
        self.cloud_pub.publish(poses)
        self.weights_pub.publish(weights)


if __name__ == "__main__":

    rospy.init_node('amcl_parser')

    # Get param
    confidence = rospy.get_param('~confidence', 0.50)

    # Run parser
    parser = AmclParser(confidence)
    rospy.spin()
    
