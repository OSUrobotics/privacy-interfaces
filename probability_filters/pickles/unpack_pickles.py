#!/usr/bin/env python

import pickle  # gonna need a pickle
import sys
import cv2
import numpy
from matplotlib import pyplot


def print_list(l):
    for el in l:
        print el

if __name__ == "__main__":

    # Parse arguments
    if len(sys.argv) < 3:
        print 'ERROR: Need two arguments!'
        exit()
    else:
        file_ar = sys.argv[1]
        print 'First argument is AR tag corners: ', file_ar
        file_filter = sys.argv[2]
        print 'Second argument is filter vertices: ', file_filter

    # Open AR tag measurements
    corners_ar = []
    with open(file_ar, 'r') as f:
        while True:
            try:
                corners_ar.append(pickle.load(f))
            except:
                break
    print len(corners_ar)
            
    # Open filter vertices
    vertices_filter = []
    with open(file_filter, 'r') as f:
        while True:
            try:
                vertices_filter.append(pickle.load(f))
            except:
                break
    print len(vertices_filter)
     
    # Convert filter vertices to bounding boxes
    corners_filter = []
    for frame in vertices_filter:
        u = [uv[0] for uv in frame[1]]
        v = [uv[1] for uv in frame[1]]
        corners_filter.append((frame[0], [(min(u), min(v)),
                                          (min(u), max(v)),
                                          (max(u), min(v)),
                                          (max(u), max(v))]))
    
    # Show how often we get each measurement type
    #pyplot.plot([el[0] - corners_ar[0][0] for el in corners_ar], [0 for el in corners_ar], 'ro')
    #pyplot.plot([el[0] - vertices_filter[0][0] for el in vertices_filter], [0 for el in vertices_filter], 'b.')
    #pyplot.show()
    
    # Sync times
    offset_max = 0.5  # seconds
    corners_filter_synced = []
    corners_ar_synced = []
    print 'Reduced data set from', len(corners_filter), '...'
    for frame in corners_filter:
        offsets = [abs(ar[0] - frame[0]) for ar in corners_ar]
        offset_lowest = min(offsets)
        if offset_lowest < offset_max:
            index = offsets.index(offset_lowest)
            corners_filter_synced.append(frame)
            corners_ar_synced.append(corners_ar[index])
        #  ... else: that entry is deleted    
    print '...to', len(corners_filter_synced)

    # Eliminate repeats
    index = 0
    print 'And again from', len(corners_filter_synced), '...'
    while index+1 < len(corners_filter_synced):
        if corners_filter_synced[index][1] == corners_filter_synced[index+1][1] and corners_ar_synced[index][1] == corners_ar_synced[index+1][1]:
            corners_filter_synced.pop(index+1)
            corners_ar_synced.pop(index+1)
        else:
            index += 1
    print '...to', len(corners_filter_synced)

    # Display rectangles on a blank image
    image = numpy.ones([480, 640, 3]) * 128
    for i in range(len(corners_ar_synced)):
        image_temp = image.copy()
        cv2.fillConvexPoly(image_temp, numpy.asarray([corners_ar_synced[i][1][0],
                                                      corners_ar_synced[i][1][2],
                                                      corners_ar_synced[i][1][3],
                                                      corners_ar_synced[i][1][1]]), (255,0,0))
        cv2.rectangle(image_temp, corners_filter_synced[i][1][0],
                      corners_filter_synced[i][1][-1],
                      (0,0,255), 2)
        cv2.imshow('Filter coverage', image_temp)
        cv2.waitKey(1000/10)  # display at 10Hz
        

