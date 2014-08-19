#!/usr/bin/env python
# 

import rospy
import numpy
import cv2
import sys
import pickle  # gonna need a pickle
from matplotlib import pyplot

UV = []

def mouse_callback(event, u, v, foo, bar):
    """ Get mouse position at 10Hz. """
    print event, u, v, foo, bar
    UV.append([u, v])
    rospy.sleep(0.10)

if __name__ == "__main__":

    rospy.init_node('make_face_trajectory')

    if len(sys.argv) < 2:
        rospy.logerr('Input the output filename as an argument!')
    else: 
        filename = sys.argv[1]

        window_name = 'Drag mouse around the screen! Press a key when done.'
        image = numpy.ones([480, 640, 3]) * 128
        cv2.namedWindow(window_name)
        cv2.imshow(window_name, image)
        cv2.setMouseCallback(window_name, mouse_callback)
        cv2.waitKey(0)
        
        with open(filename, 'w') as f:
            pickle.dump(UV, f)

        pyplot.plot([uv[0] for uv in UV], [uv[1] for uv in UV], 'ro')
        pyplot.show()
