#!/usr/bin/env python

import rospy
from amcl.msg import ParticleCloudWeights
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
from matplotlib import pyplot
from tf.transformations import *

class WeightedParticleVisualizer():
    def __init__(self):

        self.pub_markers = rospy.Publisher('/weighted_poses', MarkerArray)

        rospy.loginfo('Subscribing to cloud of weighted AMCL poses...')
        cloud_sub = message_filters.Subscriber('/particlecloud', PoseArray)
        weights_sub = message_filters.Subscriber('/particlecloud_weights', ParticleCloudWeights)
        ts = message_filters.TimeSynchronizer([cloud_sub, weights_sub], 10)
        ts.registerCallback(self.particle_filter_callback)

    def particle_filter_callback(self, poses, weights):

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
        #pyplot.plot([weight[0] for weight in weights_enumerated], '.')
        #pyplot.show()
        self.confidence = 1.0
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
            poses.poses = [poses.poses[index] for index in indices[:i+1]]
            poses.poses.append(mean_pose)  # append mean pose! 
            weights.weights = [weights.weights[index] for index in indices[:i+1]]

        markers = MarkerArray()  # clear markers
        for i in range(500):  # HACK assumes 500 poses
            marker = Marker()
            marker.header = poses.header
            marker.id = i
            marker.type = marker.ARROW
            marker.action = 0  # add/modify marker
            marker.pose = poses.poses[i]
            marker.scale.x = 0.15  # constant length
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1.0
            #if weights_enumerated[i][0] < 0.20:
            if i < 0:
                marker.color.r = 0.2
                marker.color.g = 0.2
                marker.color.b = 0.2
            #elif weights_enumerated[i][0] < 0.50:
            elif i < 5:
                marker.pose.position.z +=0.03
                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
            #elif weights_enumerated[i][0] < 1.00:
            elif i < 505:
                marker.pose.position.z +=0.03
                marker.color.r = 0
                marker.color.g = 1
                marker.color.b = 0
            if len(markers.markers) >= 501:
                markers.markers[i] = marker
            else:
                markers.markers.append(marker)

        # Add mean pose
        marker = Marker()
        marker.header = poses.header
        marker.id = i
        marker.type = marker.ARROW
        marker.action = 0  # add/modify marker
        marker.pose = mean_pose
        marker.pose.position.z += 0.07
        marker.scale.x = 0.15  # constant length
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.a = 1.0
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        if len(markers.markers) >= 501:
            markers.markers[-1] = marker
        else:
            markers.markers.append(marker)


        self.pub_markers.publish(markers)

if __name__ == "__main__":
    rospy.init_node('weighted_particle_visualizer')
    viz = WeightedParticleVisualizer()
    rospy.spin()
