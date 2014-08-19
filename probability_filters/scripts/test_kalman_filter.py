#!/usr/bin/env python
# 

import rospy
import sys
import numpy
import cv2
from probability_filters import KalmanFilter
import pickle


if __name__ == "__main__":

    rospy.init_node('test_kalman_filter')

    if len(sys.argv) < 2:
        rospy.logerr('Input the data filename as an argument!')
    else:
        filename = sys.argv[1]

        # Load fake data for testing
        with open(filename, 'r') as f:
            UV = pickle.load(f)

        # Run filter
        kf = KalmanFilter(Q = 2.0)
        image = numpy.ones([480, 640, 3]) * 128
        

        t = 0
        for i in range(100) + range(300,len(UV)):
            if rospy.is_shutdown():
                break
            #t = i * 1/10.
            t += 1/10.
            kf.run(t, UV[i])

            # Plot stuff!
            image_temp = image.copy()
            cv2.circle(image_temp, (UV[i][0], UV[i][1]), 2, (255,0,0), -1)
            cv2.circle(image_temp, (int(kf.mu[0]), int(kf.mu[1])), 5, (0,0,255))
            spread = kf.sig.diagonal()
            cv2.rectangle(image_temp, (int(kf.mu[0] - spread[0]), int(kf.mu[1] - spread[1])),
                          (int(kf.mu[0] + spread[0]), int(kf.mu[1] + spread[1])),
                          (0,0,255), 2)
            cv2.imshow('2D Position & Variance', image_temp)
            if i > 95 and i < 305:
                cv2.waitKey(0)
            else:
                cv2.waitKey(1000/100)  # display at 100Hz

