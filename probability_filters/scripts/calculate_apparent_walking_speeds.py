#!/usr/bin/env python
# 
# Uses the Pinhole Camera Model of the Asus Xtion
# to calculate how fast (pixels per second) someone
# will appear to be walking at various depths of field.
# Assumed average walking speed is 5.0km/h, or 1.39m/s.

import rospy
import os
import sys
import rosbag
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
import numpy
import cv2
from cv_bridge import CvBridge
from matplotlib import pyplot


def project_onto_depth(xyz, z_desired):
    """ Given an arbitrary vector in XYZ, scales such that Z-value is
    as desired. """
    scale = z_desired / xyz[2]
    xyz = tuple(el * scale for el in xyz)
    return xyz

if __name__ == "__main__":

    bag_path = sys.argv[1]  # path to camera_info
    bag = rosbag.Bag(bag_path)
    camera_infos = bag.read_messages(topics='/camera/rgb/camera_info')
    camera_info = camera_infos.next()[1]
    model = PinholeCameraModel()
    model.fromCameraInfo(camera_info)

    # Fig. 1: How many meters per pixel at various depths?
    print model.projectPixelTo3dRay((319.5, 239.5))
    u_neutral = 319.5
    v_neutral = 239.5
    depths = [d * 0.10 for d in range(10,50)]  # approximate cutoff distance: 4m
    res = []
    for depth in depths:
        primus = model.projectPixelTo3dRay((u_neutral, v_neutral))
        secundus = model.projectPixelTo3dRay((u_neutral+1, v_neutral))
        
        primus_at_depth = project_onto_depth(primus, depth)
        secundus_at_depth = project_onto_depth(secundus, depth)
        
        dXYZ = tuple(s - p for p,s in zip(primus_at_depth, secundus_at_depth))
        res.append([dXYZ[0], 1.39 / dXYZ[0]])

    print res
    pyplot.plot(depths, [r[1] for r in res])
    #pyplot.yscale('log')
    pyplot.title('Average Human Walking Speed (5.0 km/hr) in Pixel Space')
    pyplot.xlabel('Distance from Camera (m)')
    pyplot.ylabel('Average Walking Speed (pixels/second)')
    pyplot.show()
        
    # Fig. 2: How long to walk across FoV at various depths?
    fovs = [r[0]*639 for r in res]
    pyplot.plot(depths, [fov / 1.39 for fov in fovs])
    #pyplot.yscale('log')
    pyplot.title('')
    pyplot.xlabel('Distance from Camera (m)')
    pyplot.ylabel('Time to Walk the FoV Width at Average Speed (s)')
    pyplot.show()
        
        
            
            
            
