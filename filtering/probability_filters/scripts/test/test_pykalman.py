#!/usr/bin/env python
# 

import rospy
import sys
import numpy
import cv2
from pykalman import KalmanFilter
import pickle


if __name__ == "__main__":

    rospy.init_node('test_pykalman')

    if len(sys.argv) < 2:
        rospy.logerr('Input the data filename as an argument!')
    else:
        filename = sys.argv[1]

        # Load fake data for testing
        with open(filename, 'r') as f:
            UV = pickle.load(f)

        # Initialize filter
        kf = KalmanFilter(transition_matrices =  [[1,0],[0,1]],
                          observation_matrices = [[1,0],[0,1]])
        image = numpy.ones([480, 640, 3]) * 128

        # First two (2) measurements (initializes variables)
        measurements = numpy.ma.asarray(UV)  # masking enabled!
        measurements = numpy.ma.concatenate((numpy.ma.asarray([[0,0],[0,0]]),
                                             measurements))
        measurements[0:2] = numpy.ma.masked
        measurements[::2] = numpy.ma.masked
        measurements[100:110] = numpy.ma.masked
        measurements[200:210] = numpy.ma.masked
        measurements[300:310] = numpy.ma.masked
        measurements[400:410] = numpy.ma.masked
        mu, sig = kf.filter(measurements[0:2])  # initialize on bogus data
        print measurements
        print mu
        print sig
        print ''
        
        #t = 0
        mu = mu[-1]
        sig = sig[-1]
        for i in range(2, len(measurements)):  # starting with third measurement...

            if rospy.is_shutdown():  # easy interruption
                break

            t = i * 1/10.
            #t += 1/10.
            measurement = measurements[i]
            #print measurement
            #print mu
            #print sig
            mu, sig = kf.filter_update(mu, sig, observation=measurement)
            #print mu
            #print sig
            #print ''

            image_temp = image.copy()
            cv2.circle(image_temp, (measurement.data[0], measurement.data[1]), 2, (255,0,0), -1)
            cv2.circle(image_temp, (int(mu[0]), int(mu[1])), 5, (0,0,255))
            spread = numpy.sqrt(sig.diagonal()) * 10
            cv2.rectangle(image_temp, (int(mu[0] - spread[0]), int(mu[1] - spread[1])),
                          (int(mu[0] + spread[0]), int(mu[1] + spread[1])),
                          (0,0,255), 2)
            cv2.imshow('2D Position & Variance', image_temp)
            cv2.waitKey(1000/100)  # display at 100Hz
