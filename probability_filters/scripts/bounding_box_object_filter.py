#!/usr/bin/env python

import rospy

# Listen to /private_object tf frame
# Subscribe to /object_bounds (PolygonStamped)
# Subscribe to /camera_pose_extremes (perhaps a Float32MultiArray)

# 3) Convert <x, y, a> to <r, theta, yaw>
# 4) Choose four corner poses (each containing one of r_min, r_max, yaw_min, yaw_max)
# 5) Project our PolygonStamped onto those four poses
# 6) (Run convex hull and then) plot the rectangle (polygon)

# Publish /image_out (Image)
