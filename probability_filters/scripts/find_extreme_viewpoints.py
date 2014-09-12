#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import numpy
import tf
from tf.transformations import *
from math import atan2, sin, cos, pi


class ExtremeViewpointFinder():

    def __init__(self):
        
        self.lis = tf.TransformListener()

        self.pub = rospy.Publisher('/camera_viewpoint_extremes', PointCloud)

        rospy.Subscriber('/robot_pose_ranges', Float32MultiArray, self.ranges_callback)


    def ranges_callback(self, ranges):

        # Parse input
        x_min, x_max, y_min, y_max, w_min, w_max = ranges.data
        dx = (x_max - x_min) / 49
        dy = (y_max - y_min) / 49
        dw = (w_max - w_min) / 49

        # Generate XYW pose possibilities -- 125,000 of them!
        resolution = 50
        x_list = numpy.array([x_min + dx*step for step in range(resolution)])
        y_list = numpy.array([y_min + dy*step for step in range(resolution)])
        w_list = numpy.array([w_min + dw*step for step in range(resolution)])
        views = numpy.zeros((resolution**3, 3))
        views[:, 2] = numpy.tile(w_list, resolution**2)
        views[:, 1] = numpy.tile(numpy.repeat(y_list, resolution), resolution)
        views[:, 0] = numpy.repeat(x_list, resolution**2)
        
        # Get object location in /map frame
        obj, rotation = self.lis.lookupTransform('/map', '/private_object', rospy.Time(0))

        ###### Convert to RTY ######
        # Calculate radius
        dx = obj[0] - views[:, 0]
        dy = obj[1] - views[:, 1]
        r = numpy.sqrt(dx**2 + dy**2)
        offset = numpy.array([dx, dy])  # vectors
        offset /= r  # normalize

        # Calculate theta (angle of vector from object to robot)
        theta = numpy.arctan2(-1 * offset[1, :], -1 * offset[0, :])  # positive is counter-clockwise

        # Calculate all heading vectors!
        headings = numpy.array([numpy.cos(views[:, 2]), 
                                numpy.sin(views[:, 2])])

        # Calculate camera yaw (angle from gaze vector to object line-of-sight vector)
        cosine = numpy.sum(headings * offset, axis=0)  # calculate dot product by hand
        sine = headings[0, :] * offset[1, :] - headings[1, :] * offset[0, :]  # calculate the cross product by hand
        yaw = -1 * numpy.arctan2(sine, cosine)  # positive is counter-clockwise

        ###### Select corner poses ######
        # Remove poses not facing the object
        facing_object = numpy.logical_and(yaw > -pi/2, yaw < pi/2)
        r = r[facing_object]
        theta = theta[facing_object]
        yaw = yaw[facing_object]

        # Find min/max of radius & yaw
        r_min = min(r); r_max = max(r)
        yaw_min = min(yaw); yaw_max = max(yaw)

        ###### Switch to the POV of the Camera ######
        # Permute (yields 4 poses), convert, and transform
        base_views = PointCloud()
        base_views.header.frame_id = '/base_footprint'
        base_views.header.stamp = rospy.Time(0)
        for r_i in [r_min, r_max]:
            for yaw_i in [yaw_min, yaw_max]:
                base_view = Point32()
                base_view.x = r_i * cos(-yaw_i)
                base_view.y = r_i * sin(-yaw_i)
                base_view.z = obj[2] - 0.0  # since base_footprint is at z = 0
                base_views.points.append(base_view)
        camera_views = self.lis.transformPointCloud('/camera_rgb_optical_frame', base_views)

        self.pub.publish(camera_views)


        

if __name__ == "__main__":

    rospy.init_node('extreme_viewpoint_finder')

    # Run finder
    finder = ExtremeViewpointFinder()
    rospy.spin()
    
