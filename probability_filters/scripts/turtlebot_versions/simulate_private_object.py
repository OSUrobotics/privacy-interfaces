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
        self.object_location = ((-0.55482, -1.1364, 0.46802),
                                (0.61632, 0.32777, 0.63445, -0.33195))


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
        self.br.sendTransform((-0.04704077773999993, 0.18815397351999996, 0.301379371159),
                              (0.40099994, -0.71826226,  0.47798316, -0.30794739),
                              rospy.Time.now(),
                              '/camera',
                              '/map')
        self.pub_polygon.publish(self.polygon)

    def get_object_location(self):
        """ Transform AR Tag location into map frame. """
        frame_id = '/ar_marker_203'
        target_frame = '/map'
        while not self.lis.frameExists(frame_id):
            rospy.sleep(0.1)
            print 'Waiting!'
        self.lis.waitForTransform(target_frame, frame_id, rospy.Time(0), rospy.Duration(2.0))
        translation, rotation = self.lis.lookupTransform(target_frame, frame_id, rospy.Time(0))
        return translation, rotation
        
    def update_polygon(self, w, h):
        self.polygon = PolygonStamped()  # erase old polygon
        self.polygon.header.frame_id = '/private_object'
        self.polygon.header.stamp = rospy.Time.now()
        for i in range(4):  # add four vertices
            self.polygon.polygon.points.append(Point32())
        self.polygon.polygon.points[0].x = -1 * h * 3/5
        self.polygon.polygon.points[0].y = -1 * w / 2 
        self.polygon.polygon.points[1].x =      h * 2/5 
        self.polygon.polygon.points[1].y = -1 * w / 2
        self.polygon.polygon.points[2].x =      h * 2/5
        self.polygon.polygon.points[2].y =      w / 2
        self.polygon.polygon.points[3].x = -1 * h * 3/5
        self.polygon.polygon.points[3].y =      w / 2


        # Transform into /map frame
        """
        for i in range(len(self.polygon.polygon.points)):
            vs = Vector3Stamped()
            vs.header.frame_id = '/private_object'
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

        print self.polygon.polygon
        """

if __name__ == "__main__":

    rospy.init_node('simulate_private_object')
    obj = SimulatedObject()
    obj.run()
