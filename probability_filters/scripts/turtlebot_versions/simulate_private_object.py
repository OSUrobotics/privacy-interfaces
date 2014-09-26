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
        #print self.object_location
        self.object_location = ((-0.56316, -1.1028, 0.46449),
                                (0.61612, 0.28192, 0.66879, -0.30602))


        r = rospy.Rate(30)
        i = 0
        while not rospy.is_shutdown():
            self.update_polygon(0.215, 0.245)
            self.send()
            r.sleep()

    def send(self):
        """ Broadcast TF frames and Publish stuff. """
        #self.br.sendTransform(self.object_location,
        self.br.sendTransform(self.object_location[0],
                              self.object_location[1],
                              rospy.Time.now(),
                              '/private_object',
                              '/map')
        self.pub_polygon.publish(self.polygon)

    def get_object_location(self):
        """ Transform AR Tag location into map frame. """
        frame_id = 'ar_marker_203'
        target_frame = '/map'
        #if self.lis.frameExists(frame_id):
        self.lis.waitForTransform(target_frame, frame_id, rospy.Time.now(), rospy.Duration(5.0))
        translation, rotation = self.lis.lookupTransform(target_frame, 'ar_marker_203', rospy.Time.now())
        return translation, rotation
        
    def update_polygon(self, w, h):
        self.polygon = PolygonStamped()  # erase old polygon
        self.polygon.header.frame_id = '/private_object'
        self.polygon.header.stamp = rospy.Time.now()
        for i in range(4):  # add four vertices
            self.polygon.polygon.points.append(Point32())
        self.polygon.polygon.points[0].x = -1 * w / 2
        self.polygon.polygon.points[0].y = -1 * h / 2
        self.polygon.polygon.points[1].x =      w / 2
        self.polygon.polygon.points[1].y = -1 * h / 2
        self.polygon.polygon.points[2].x =      w / 2
        self.polygon.polygon.points[2].y =      h / 2
        self.polygon.polygon.points[3].x = -1 * w / 2
        self.polygon.polygon.points[3].y =      h / 2
        
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
