#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud

def faces_cb(faces):
    rospy.loginfo('Got some faces! Maybe!')

rospy.init_node('pull_faces')

rospy.Subscriber('/face_detector/faces_cloud', PointCloud, faces_cb)
rospy.loginfo('Subscribed to faces!')
rospy.spin()
