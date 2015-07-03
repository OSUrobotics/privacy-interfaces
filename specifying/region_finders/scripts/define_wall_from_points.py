#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped  # for RViz /clicked_point
from visualization_msgs.msg import Marker
import copy

class WallMaker():
    def __init__(self, point_topic):
        self.points = []
        self.wall_height = 3.0  # meters
        self.init_marker()

        self.publisher = rospy.Publisher('/zones/walls/region_0/marker', Marker)

        rospy.Subscriber(point_topic, PointStamped, self.point_callback)

        self.publish_marker()

    def init_marker(self):
        self.marker = Marker()
        self.marker.header.frame_id = '/camera_rgb_optical_frame'
        self.marker.id = 0
        self.marker.type = Marker.TRIANGLE_LIST
        self.marker.action = Marker.ADD
        self.marker.pose.orientation.w = 1.0
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0
        self.marker.color.a = 1.0

    def point_callback(self, point):
        if not self.points:  # if this is our first point ...
            point_fake = copy.deepcopy(point.point)
            point_fake.x += 0.10  # ... make a fake wall segment 10cm wide for visual aid
            self.points.append(point_fake)
        self.points.append(point.point)
        self.update_marker()

    def update_marker(self):
        self.marker.header.stamp = rospy.Time.now()
        wall_bottom_old = self.points[-2]
        wall_top_old = copy.deepcopy(wall_bottom_old); wall_top_old.y -= self.wall_height
        wall_bottom_new = self.points[-1]
        wall_top_new = copy.deepcopy(wall_bottom_new); wall_top_new.y -= self.wall_height

        # Add first triangle
        self.marker.points.append(wall_bottom_old)
        self.marker.points.append(wall_top_old)
        self.marker.points.append(wall_top_new)

        # Add second triangle
        self.marker.points.append(wall_bottom_new)
        self.marker.points.append(wall_bottom_old)
        self.marker.points.append(wall_top_new)

    def publish_marker(self):
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.publisher.publish(self.marker)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('define_wall_from_points')
    wall_maker = WallMaker('/clicked_point')
    
