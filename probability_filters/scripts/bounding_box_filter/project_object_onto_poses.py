#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PolygonStamped, PoseArray
import numpy
import tf
from tf.transformations import *
from math import atan2, sin, cos, pi, sqrt


class ObjectProjector():

    def __init__(self):
        
        self.need_bounds = True
        
        self.lis = tf.TransformListener()

        self.pub = rospy.Publisher('/object_vertex_projections', PointCloud)

        rospy.Subscriber('/object_bounds_map', PolygonStamped, self.bounds_callback)
        
        while self.need_bounds:  # block until we get object boundaries
            rospy.sleep(0.01)

        rospy.loginfo('Got object bounds! Proceeding to find extreme projections.')

        rospy.Subscriber('/particlecloud_select', PoseArray, self.poses_callback)
        

    def bounds_callback(self, bounds):
        if self.need_bounds:
            self.bounds = bounds
            self.need_bounds = False


    def poses_callback(self, poses):

        # Get object location in /map frame
        obj, rotation = self.lis.lookupTransform('/map', '/private_object', rospy.Time(0))

        ###### Switch to the POV of the Camera ######
        base_views = PointCloud()
        base_views.header.frame_id = '/base_footprint'
        base_views.header.stamp = rospy.Time(0)
        for pose in poses.poses:
            # Get pose orientation angle about z-axis
            q = [pose.orientation.x,
                 pose.orientation.y,
                 pose.orientation.z,
                 pose.orientation.w]
            x_rotation, y_rotation, z_rotation = euler_from_quaternion(q)
            w = z_rotation

            for point in self.bounds.polygon.points:  # resolve object bounds here because it'd be costly to do it 125,000 times

                d = [pose.position.x - obj[0] - point.x,  # viewpoint relative to object *vertex*, /map frame
                     pose.position.y - obj[1] - point.y]
                    
                ###### Convert to RTY ######
                # Calculate radius
                r = numpy.sqrt(d[0]**2 + d[1]**2)
                offset = [el/r for el in d]  # normalize
                # Calculate theta (angle of vector from object to robot)
                theta = numpy.arctan2(-1 * d[1], -1 * d[0])  # positive is counter-clockwise
                # Calculate all heading vectors!
                heading = [cos(w), sin(w)]
                # Calculate camera yaw (angle from gaze vector to object line-of-sight vector)
                cosine = sum([h * o for h, o in zip(heading, offset)])  # calculate dot product by hand
                sine = heading[0] * offset[1] - heading[1] * offset[0]  # calculate the cross product by hand
                yaw = -1 * atan2(sine, cosine)  # positive is counter-clockwise
    
                if yaw > -pi/2 and yaw < pi/2:  # abort if we're not facing the object vertex
                    
                    # Object *vertex* location from perspective of /base_footprint
                    base_view = Point32()
                    base_view.x = r * cos(-yaw)
                    base_view.y = r * sin(-yaw)
                    base_view.z = obj[2] + point.z - 0.0  # since base_footprint is at z = 0
                    base_views.points.append(base_view)

        camera_views = self.lis.transformPointCloud('/map', base_views)  # transform to map frame

        camera_views.header.stamp = rospy.Time.now()  # close enough
        self.pub.publish(camera_views)
        

if __name__ == "__main__":

    rospy.init_node('object_projector')

    # Run projector
    finder = ObjectProjector()
    rospy.spin()
    
