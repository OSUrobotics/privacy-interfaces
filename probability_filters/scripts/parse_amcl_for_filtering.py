#!/usr/bin/env python

import rospy


# Inputs: use_particles = False, sigmas = 2

# Subscribe to /amcl_pose for PoseWithCovariance and to /particlecloud for PoseArray

# 0) Shift from the /base_link or whatever to /camera_rgb_optical_frame
# 1) Get min & max values for each degree of freedom (e.g. x, y, yaw)
# 2) Permute into, here, 8 triples

# Publish /camera_pose_extremes as, e.g., Float32MultiArray
