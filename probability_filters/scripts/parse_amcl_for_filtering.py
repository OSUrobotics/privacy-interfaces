#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseArray
import numpy
import tf
from tf.transformations import *


class AmclParser():

    def __init__(self, use_particles=False, confidence_in_sigmas=2):

        self.conf = confidence_in_sigmas  # confidence level

        self.lis = tf.TransformListener()

        self.pub = rospy.Publisher('/camera_pose_extremes', PoseArray)

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
        
        # Form PoseArray in camera frame; publish!
        camera_poses = permute_pose_ranges_2d([[x_min, x_max], 
                                               [y_min, y_max], 
                                               [w_min, w_max]])
        camera_poses.header.frame_id = poses.header.frame_id
        camera_poses.header.stamp = rospy.Time.now()
        self.pub.publish(camera_poses)


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

        # Form PoseArray in camera frame; publish!
        camera_poses = self.permute_pose_ranges_2d([[x_min, x_max], 
                                               [y_min, y_max], 
                                               [w_min, w_max]])
        camera_poses.header.frame_id = pose.header.frame_id
        camera_poses = self.transform_pose_array(camera_poses, 
                                                 '/camera_rgb_optical_frame',
                                                 '/base_link')
        camera_poses.header.stamp = rospy.Time.now()
        self.pub.publish(camera_poses)


    def permute_pose_ranges_2d(self, ranges):
        """ INPUT ranges = [[x_min, x_max], [y_min, y_max], [w_min, w_max]] """
        poses = PoseArray()
        for x in ranges[0]:
            for y in ranges[1]:
                for w in ranges[2]:
                    pose = Pose()
                    [pose.position.x,
                     pose.position.y,
                     pose.position.z] = [x, y, 0.0]
                    [pose.orientation.x,
                     pose.orientation.y,
                     pose.orientation.z,
                     pose.orientation.w] = quaternion_from_euler(0.0, 0.0, w)
                    poses.poses.append(pose)
        return poses
            
    def transform_pose_array(self, pose_array, source_frame, target_frame):

        # Look up transform from [base] to [camera]
        translation, rotation = self.lis.lookupTransform(target_frame, source_frame, rospy.Time(0))

        for pose in pose_array.poses:

            # Translate
            pose.position.x += translation[0]
            pose.position.y += translation[1]
            pose.position.z += translation[2]

            # Rotate
            [pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w] = quaternion_multiply([pose.orientation.x,
                                                        pose.orientation.y,
                                                        pose.orientation.z,
                                                        pose.orientation.w], rotation)

        return pose_array
        

if __name__ == "__main__":

    rospy.init_node('amcl_parser')

    # Get params
    use_particles = rospy.get_param('~use_particles', False)
    confidence_in_sigmas = rospy.get_param('~confidence_in_sigmas', 2)

    # Run parser
    parser = AmclParser(use_particles, confidence_in_sigmas)
    rospy.spin()
    
