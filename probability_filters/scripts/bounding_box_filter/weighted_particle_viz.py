#!/usr/bin/env python

import rospy
from amcl.msg import ParticleCloudWeights
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray
import message_filters

class WeightedParticleVisualizer():
    def __init__(self):

        self.pub_markers = rospy.Publisher('/weighted_poses', MarkerArray)

        rospy.loginfo('Subscribing to cloud of weighted AMCL poses...')
        cloud_sub = message_filters.Subscriber('/particlecloud', PoseArray)
        weights_sub = message_filters.Subscriber('/particlecloud_weights', ParticleCloudWeights)
        ts = message_filters.TimeSynchronizer([cloud_sub, weights_sub], 10)
        ts.registerCallback(self.particle_filter_callback)

    def particle_filter_callback(self, poses, weights):

        w = [weight.data for weight in weights.weights]
        print min(w), max(w)

        # Use Decorate-Sort-Undecorate idiom
        weights_enumerated = [[weight.data, i] for i, weight in enumerate(weights.weights)]
        weights_enumerated.sort(reverse=True)
        for i in range(len(weights_enumerated)):
            if i > 0:
                weights_enumerated[i][0] += weights_enumerated[i-1][0]  # add previous weight
        indices = [el[1] for el in weights_enumerated]
        poses.poses = [poses.poses[index] for index in indices]
        weights.weights = [weights.weights[index] for index in indices]

        

        markers = MarkerArray()  # clear markers
        for i in range(500):  # HACK assumes 500 poses
            marker = Marker()
            marker.header = poses.header
            marker.id = i
            marker.type = marker.ARROW
            marker.action = 0  # add/modify marker
            marker.pose = poses.poses[i]
            marker.scale.x = 0.10  # constant length
            marker.scale.y = 0.01
            marker.scale.z = 0.01
            marker.color.a = 1.0
            if weights_enumerated[i][0] < 0.20:
                marker.color.r = 1.0
                marker.color.g = 0
                marker.color.b = 0
            elif weights_enumerated[i][0] < 0.50:
                marker.color.r = 0
                marker.color.g = 0.5
                marker.color.b = 0
            elif weights_enumerated[i][0] < 1.00:
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 0.2
            if len(markers.markers) >= i + 1:
                markers.markers[i] = marker
            else:
                markers.markers.append(marker)

        self.pub_markers.publish(markers)

if __name__ == "__main__":
    rospy.init_node('weighted_particle_visualizer')
    viz = WeightedParticleVisualizer()
    rospy.spin()
