#!/usr/bin/env python

# TEST PLATFORM for step #3 below.
# 3) Convert <x, y, a> to <r, theta, yaw>
# 4) Choose four corner poses (each containing one of r_min, r_max, yaw_min, yaw_max)
# 5) Project our PolygonStamped onto those four poses
# 6) (Run convex hull and then) plot the rectangle (polygon)

import rospy
import numpy
import random
from math import sqrt, atan2
from tf.transformations import quaternion_about_axis, quaternion_matrix
from matplotlib import pyplot


# Premise: object is at (0, 0) and robot has random (x, y, theta) centered at (0, 0, 0)
if __name__ == '__main__':

    # Locate object
    obj = [0.0, 0.0, 0.0]

    # Locate robots
    dev = 5.0
    robots_xyw = []
    for i in range(1000):
        robot = [random.gauss(0.0, dev),
                 random.gauss(0.0, dev),
                 random.gauss(0.0, dev)]
        robots_xyw.append(robot)
    
    # Add robots for permuted min & max bounds
    x = [robot[0] for robot in robots_xyw]
    y = [robot[1] for robot in robots_xyw]
    w = [robot[2] for robot in robots_xyw]
    ranges = [[min(x), max(x)], [min(y), max(y)], [min(w), max(w)]]
    for x_i in ranges[0]:
        for y_i in ranges[1]:
            for w_i in ranges[2]:
                robots_xyw.append([x_i, y_i, w_i])
    

    # Do conversions
    robots_rty = []
    for robot in robots_xyw:

        # Calculate radius
        dx = obj[0] - robot[0]
        dy = obj[1] - robot[1]
        r = sqrt(dx**2 + dy**2)
        offset = [dx, dy, 0.0]  # vector
        offset = [el / r for el in offset]  # normalize

        # Calculate theta (angle of vector from object to robot)
        theta = atan2(-1 * offset[1], -1 * offset[0])  # positive is counter-clockwise

        # Calculate heading vector of robot
        q_robot = quaternion_about_axis(robot[2], (0, 0, 1))
        R_robot = quaternion_matrix(q_robot)
        heading_robot = numpy.matrix([1, 0, 0, 1]) * numpy.matrix(R_robot).I
        heading_robot /= heading_robot[0, 3]  # ensure homogeneity isn't messing stuff up
        heading_robot = heading_robot[0, 0:3].tolist()[0]  # convert from homogeneous to...not

        # Calculate camera yaw (angle from gaze vector to object line-of-sight vector)
        cosine = numpy.dot(heading_robot, offset)
        cross = numpy.cross(heading_robot, offset)
        sine = cross[2]
        yaw = -1 * atan2(sine, cosine)  # positive is counter-clockwise

        robots_rty.append([r, theta, yaw])
    

    robots_xywrty = [xyz + rty for xyz, rty in zip(robots_xyw, robots_rty)]
    radii = [robot[3] for robot in robots_xywrty]
    yaws = [robot[5] for robot in robots_xywrty]
    for robot in robots_xywrty:
        print robot
    print min(radii), max(radii)
    print min(yaws), max(yaws)
    # RESULT: extremes in XYW do *not* yield the extremes in RTY
