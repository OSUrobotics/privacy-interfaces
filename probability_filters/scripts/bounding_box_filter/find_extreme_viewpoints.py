#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PolygonStamped
import numpy
import tf
from tf.transformations import *
from math import atan2, sin, cos, pi, sqrt


class ExtremeViewpointFinder():

    def __init__(self):
        
        self.need_bounds = True
        
        self.lis = tf.TransformListener()

        self.pub = rospy.Publisher('/extreme_vertex_projections', PointCloud)

        rospy.Subscriber('/object_bounds_map', PolygonStamped, self.bounds_callback)
        
        while self.need_bounds:  # block until we get object boundaries
            rospy.sleep(0.01)

        rospy.loginfo('Got object bounds! Proceeding to find extreme projections.')

        rospy.Subscriber('/robot_pose_ranges', Float32MultiArray, self.ranges_callback)
        

    def bounds_callback(self, bounds):
        if self.need_bounds:
            self.bounds = bounds
            self.need_bounds = False


    def ranges_callback(self, ranges):

        # Parse input
        x_min, x_max, y_min, y_max, w_min, w_max = ranges.data
        resolution = 2
        dx = (x_max - x_min) / (resolution-1)
        dy = (y_max - y_min) / (resolution-1)
        dw = (w_max - w_min) / (resolution-1)

        # Generate XYW pose possibilities -- 125,000 of them!
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

        # Find min/max of radius, theta, & yaw
        r_min = min(r); r_max = max(r)
        theta_min = min(theta); theta_max = max(theta)
        yaw_min = min(yaw); yaw_max = max(yaw)
        #print [r_min, r_max], [theta_min, theta_max], [yaw_min, yaw_max]

        ###### Switch to the POV of the Camera ######
        # Permute (yields 4 poses), convert, and transform
        base_views = PointCloud()
        base_views.header.frame_id = '/base_footprint'
        base_views.header.stamp = rospy.Time(0)
        for r_i in [r_min, r_max]:
            for theta_i in [theta_min, theta_max]:
                for yaw_i in [yaw_min, yaw_max]:
                    for point in self.bounds.polygon.points:  # resolve object bounds here because it'd be costly to do it 125,000 times

                        base = [r_i * cos(theta_i),  # viewpoint relative to object *center*, /map frame
                                r_i * sin(theta_i)]

                        base_rel = [b - p for b, p in zip(base, [point.x,  # viewpoint relative to object *vertex*, /map frame
                                                                 point.y])]
                        

                        # Calculate new radius and yaw (relative to object vertex instead of center)
                        r_new = sqrt(sum([el**2 for el in base_rel]))
                        base = [el / r_i for el in base]  # normalize
                        base_rel = [el / r_new for el in base_rel]  # normalize
                        cosine = sum([b * br for b, br in zip(base, base_rel)])  # calculate dot product by hand
                        sine = base[0] * base_rel[1] - base[1] * base_rel[0]  # calculate cross product by hand
                        d_yaw = -1 * atan2(sine, cosine)
                        yaw_new = yaw_i + d_yaw

                        if yaw_new > -pi/2 and yaw_new < pi/2:  # abort if we're not facing the object vertex

                            # Object *vertex* location from perspective of /base_footprint
                            base_view = Point32()
                            base_view.x = r_new * cos(-yaw_new)
                            base_view.y = r_new * sin(-yaw_new)
                            base_view.z = obj[2] + point.z - 0.0  # since base_footprint is at z = 0
                            base_views.points.append(base_view)

        camera_views = self.lis.transformPointCloud('/map', base_views)  # transform to map frame

        camera_views.header.stamp = rospy.Time.now()  # close enough
        self.pub.publish(camera_views)
        #self.pub.publish(base_views)  # FOR DEBUG ONLY
        

if __name__ == "__main__":

    rospy.init_node('extreme_viewpoint_finder')

    # Run finder
    finder = ExtremeViewpointFinder()
    rospy.spin()
    
