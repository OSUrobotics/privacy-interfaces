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

        # Construct mean pose
        how_many = len(poses.poses)
        mean_pose = Pose()
        RZ = 0
        for pose in poses.poses:
            mean_pose.position.x += pose.position.x / how_many
            mean_pose.position.y += pose.position.y / how_many
            mean_pose.position.z += pose.position.z / how_many
            q = [pose.orientation.x,
                 pose.orientation.y,
                 pose.orientation.z,
                 pose.orientation.w]
            rx, ry, rz = euler_from_quaternion(q)
            RZ += rz / how_many
        [mean_pose.orientation.x,
         mean_pose.orientation.y,
         mean_pose.orientation.z,
         mean_pose.orientation.w] = quaternion_about_axis(RZ, (0,0,1))
        
        # Use Decorate-Sort-Undecorate idiom
        weights_sum = sum([weight.data for weight in weights.weights])
        #print weights_sum
        weights_enumerated = [[weight.data / weights_sum, i] for i, weight in enumerate(weights.weights)]
        weights_enumerated.sort(reverse=True)
        if self.confidence == -1:  # just the mean pose
            poses.poses = [mean_pose]
        else:
            for i in range(len(weights_enumerated)):
                if i > 0:
                    weights_enumerated[i][0] += weights_enumerated[i-1][0]  # add previous weight
                if weights_enumerated[i][0] > self.confidence:
                    rospy.loginfo('Selected {0} poses.'.format(i+1))
                    break
            indices = [el[1] for el in weights_enumerated]
            #poses.poses = [poses.poses[index] for index in indices[:i+1]]
            poses.poses = [poses.poses[index] for index in indices[:]]  # HACK for figure
            poses.poses.append(mean_pose)  # append mean pose! 
            weights.weights = [weights.weights[index] for index in indices[:i+1]]
            self.weights_pub.publish(weights)
        self.cloud_pub.publish(poses)


if __name__ == "__main__":

    rospy.init_node('amcl_parser')

    # Get param
    confidence = rospy.get_param('~confidence', 0.50)

    # Run parser
    parser = AmclParser(confidence)
    rospy.spin()
    
