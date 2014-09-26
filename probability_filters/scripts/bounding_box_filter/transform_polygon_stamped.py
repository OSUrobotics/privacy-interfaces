#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point32, PolygonStamped, Vector3Stamped
import tf


class PolygonTransformer():
    def __init__(self, target_frame):
        self.target_frame = target_frame
        self.lis = tf.TransformListener()
        self.pub = rospy.Publisher('/polygon_out', PolygonStamped)
        rospy.Subscriber('/polygon_in', PolygonStamped, self.transform)
        
    def transform(self, polygon):
        
        v = Vector3Stamped()
        v.header = polygon.header
        self.lis.waitForTransform(self.target_frame, 
                                  v.header.frame_id, 
                                  v.header.stamp, 
                                  rospy.Duration(3.0))
        for point in polygon.polygon.points:
            [v.vector.x,
             v.vector.y,
             v.vector.z] = [point.x,
                            point.y,
                            point.z]
            v_tf = self.lis.transformVector3(self.target_frame, v)
            [point.x,
             point.y,
             point.z] = [v.vector.x,
                         v.vector.y,
                         v.vector.z]
        polygon.header.frame_id = self.target_frame
        self.pub.publish(polygon)

    
if __name__ == '__main__':
    rospy.init_node('polygon_transformer')
    target_frame = rospy.get_param('target_frame', '/map')
    transformer = PolygonTransformer(target_frame)
    rospy.spin()

