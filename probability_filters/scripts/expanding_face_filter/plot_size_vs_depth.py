#!/usr/bin/env python
# 
# PLOTS a series of faces at different depths
# to estimate a relation between face size and depth.
# Ultimate goal: window_expansion = f(dt, depth(face_size), confidence, camera_info)

import rospy
import rosbag
from image_geometry import PinholeCameraModel
import os
import sys
import numpy
from matplotlib import pyplot
import pickle  # gonna need a pickle

        
if __name__ == "__main__":

    rospy.init_node('plot_size_vs_depth')

    path_root = '/home/ruebenm/workspaces/privacy_ws/src/probability_filters/pickles/'

    bag = rosbag.Bag(path_root + '../bags/asus_xtion_camera_info.bag')
    infos = bag.read_messages(topics='/camera/rgb/camera_info')
    info = infos.next()[1]

    model = PinholeCameraModel()
    model.fromCameraInfo(info)

    paths = [path_root + 'cameron-00.pickle',
             path_root + 'matt-00.pickle',
             path_root + 'jasper-00.pickle']

    for path, color, name in zip(paths, 'kbg', ['Cameron', 'Matthew', 'Jasper']):
        with open(path, 'r') as f:
            data = pickle.load(f)
            depths = [d[0] for d in data]
            sizes_pixel = [d[1][-1] for d in data]
            top_lefts = [model.projectPixelTo3dRay((u,v))
                           for (d, (u,v,w,h)) in [(d[0], d[1]) for d in data]]
            top_rights = [model.projectPixelTo3dRay((u+w,v))
                           for (d, (u,v,w,h)) in [(d[0], d[1]) for d in data]]
            sizes_meter = [tr[0] / tr[2] * d - tl[0] / tl[2] * d
                           for tl, tr, d in zip(top_lefts, top_rights, depths)]
            size_avg = sum(sizes_meter) / len(sizes_meter)
            print size_avg

            depths_true = [i*0.01 for i in range(50,501)]
            sizes_pixel_true = [model.project3dToPixel((size_avg, 0, d))[0]-
                                model.project3dToPixel((0, 0, d))[0]
                                for d in depths_true]

            pyplot.plot(sizes_pixel, depths, color+'o', label=name+' Actual')
            pyplot.plot(sizes_pixel_true, depths_true, color, label=name+' Calculated, Size = '+str(int(size_avg*1000))+'mm')
            pyplot.title('Face Sizes at different Depths -- Average NAW Male Face Size = 187mm')
            pyplot.xlabel('Face Size (pixels)')
            pyplot.ylabel('Depth (meters)')

    pyplot.legend(loc='right')
    pyplot.show()
