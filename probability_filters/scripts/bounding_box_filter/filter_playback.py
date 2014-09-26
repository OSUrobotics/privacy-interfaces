#!/usr/bin/env python

# Opens pickled filter extents and plots them on bagged images

import rospy
import sys
import os
import tf
import cv2
import numpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud
from image_geometry import PinholeCameraModel
from cv_bridge import CvBridge
import pickle  # gonna need a pickle

import rosbag

if __name__ == '__main__':

    bridge = CvBridge()

    bag_file = sys.argv[1]
    pickle_file = sys.argv[2]

    # Pop open that pickle jar
    corners = []
    with open(pickle_file, 'r') as f:
        while True:
            try:
                corners.append(pickle.load(f))
            except (EOFError, pickle.UnpicklingError):
                break
    print len(corners)

    # Break open that bag
    bag = rosbag.Bag(bag_file, 'r')
    for topic, msg, t in bag.read_messages(topics=['/camera/rgb/image_color']):
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Find nearest set of corners
        time = msg.header.stamp.to_time()
        dt = [abs(frame[0] - time) for frame in corners]
        if min(dt) < 1.0:  # within 1/5 of a second
            index = dt.index(min(dt))
            vertices = corners[index][1]
            uv_convex = cv2.convexHull(numpy.array(vertices))  # convex hull algorithm
            cv2.fillConvexPoly(image, uv_convex, (255, 0, 0))

        cv2.imshow('Image with AMCL Filtering', image)
        cv2.waitKey(100)

