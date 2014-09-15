#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point32, PolygonStamped, Vector3Stamped
import tf
from math import pi


class SimulatedObject():
    def __init__(self):

        self.lis = tf.TransformListener()

        # Init publishers and broadcasters
        self.br = tf.TransformBroadcaster()
        self.pub_polygon = rospy.Publisher('/object_bounds', PolygonStamped)

    def run(self):
        """ Continuously send out information, as if an object is detected. """
        rospy.loginfo('Running object simulator...now!')
        
        #self.object_location = self.get_object_location()

        r = rospy.Rate(30)
        i = 0
        while not rospy.is_shutdown():
            self.update_polygon(0.15)
            self.send()
            r.sleep()

    def send(self):
        """ Broadcast TF frames and Publish stuff. """
        #self.br.sendTransform(self.object_location,
        self.br.sendTransform((3.30, 1.35, 0.45),
                              (0.0, 0.0, 0.0, 1.0),  # rotation doesn't matter
                              rospy.Time.now(),
                              '/private_object',
                              '/map')
        self.pub_polygon.publish(self.polygon)

    def get_object_location(self):
        """ Transform object location from robot frame to map frame. """
        obj_loc = PointStamped()
        obj_loc.header.frame_id = '/camera_rgb_optical_frame'
        obj_loc.header.stamp = rospy.Time(0)
        obj_loc.point.x =  0.0
        obj_loc.point.y = -0.2
        obj_loc.point.z =  1.5

        map_frame = '/map'
        self.lis.waitForTransform(obj_loc.header.frame_id, 
                                  map_frame,
                                  rospy.Time(0), 
                                  rospy.Duration(3.0))
        obj_loc_pt = self.lis.transformPoint(map_frame, obj_loc)
        obj_loc_tuple = (obj_loc_pt.point.x,
                         obj_loc_pt.point.y,
                         obj_loc_pt.point.z)

        return obj_loc_tuple
        
    def update_polygon(self, s):
        self.polygon = PolygonStamped()  # erase old polygon
        self.polygon.header.frame_id = '/private_object'
        self.polygon.header.stamp = rospy.Time.now()
        for i in range(4):  # add four vertices
            self.polygon.polygon.points.append(Point32())
        r = s / 2.0  # radius = 1/2 * side length
        self.polygon.polygon.points[0].x = -1 * r
        self.polygon.polygon.points[0].y = -1 * r
        self.polygon.polygon.points[1].x =      r
        self.polygon.polygon.points[1].y = -1 * r
        self.polygon.polygon.points[2].x =      r
        self.polygon.polygon.points[2].y =      r
        self.polygon.polygon.points[3].x = -1 * r
        self.polygon.polygon.points[3].y =      r
        
        # Transform into /map frame
        for i in range(len(self.polygon.polygon.points)):
            vs = Vector3Stamped()
            vs.header.frame_id = '/camera_rgb_optical_frame'
            vs.header.stamp = rospy.Time(0)
            [vs.vector.x,
             vs.vector.y,
             vs.vector.z] = [self.polygon.polygon.points[i].x,
                             self.polygon.polygon.points[i].y,
                             self.polygon.polygon.points[i].z]
            self.lis.waitForTransform(vs.header.frame_id,
                                      '/map',
                                      rospy.Time(0),
                                      rospy.Duration(3.0))
            vs = self.lis.transformVector3('/map', vs)
            [self.polygon.polygon.points[i].x,
             self.polygon.polygon.points[i].y,
             self.polygon.polygon.points[i].z] = [vs.vector.x,
                                                  vs.vector.y,
                                                  vs.vector.z]

if __name__ == "__main__":

    rospy.init_node('simulate_private_object')
    obj = SimulatedObject()
    obj.run()
