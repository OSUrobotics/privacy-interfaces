#!/usr/bin/env python

import pickle  # gonna need a pickle
import sys
import cv2
import numpy
from matplotlib import pyplot


def print_list(l):
    for el in l:
        print el

def unpack_pickle(file_ar, file_filter):

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


    """
    # Display rectangles on a blank image
    image = numpy.ones([480, 640, 3]) * 128
    for i in range(len(corners_ar_synced)):
        image_temp = image.copy()
        cv2.fillConvexPoly(image_temp, numpy.asarray(corners_ar_synced[i][1]), (255,0,0))
        cv2.fillConvexPoly(image_temp, numpy.asarray(corners_filter_synced[i][1]), (0,0,255))
        cv2.imshow('Filter coverage', image_temp)
        cv2.waitKey(1000/10)  # display at 10Hz
    """

    # Do metrical analysis
    image = numpy.ones([480, 640, 3]) * 0
    metrics = {'time': [], 'total': 640*480, 'covered': [], 'uncovered': [], 'wasted': []}
    print 'And finally from', len(corners_ar_synced), '...'
    for i in range(len(corners_ar_synced)):
        # Check that AR tag is *completely* within image
        bad_uv = [u < 0 or u > 640 or 
                  v < 0 or v > 480  
                  for [u, v] in corners_ar_synced[i][1]]
        if not any(bad_uv):  
            image_ar = image.copy()
            cv2.fillConvexPoly(image_ar, numpy.asarray(corners_ar_synced[i][1]), (255,0,0))
            image_filter = image.copy()
            cv2.fillConvexPoly(image_filter, numpy.asarray([corners_filter_synced[i][1][0],
                                                            corners_filter_synced[i][1][1],
                                                            corners_filter_synced[i][1][3],
                                                            corners_filter_synced[i][1][2]]), (0,0,255))
            image_coverage = image_ar + image_filter
            metrics['time'].append(corners_filter_synced[i][0])
            metrics['covered'].append(numpy.sum(numpy.all(image_coverage == [255, 0, 255], axis=2)))
            metrics['uncovered'].append(numpy.sum(numpy.all(image_coverage == [255, 0, 0], axis=2)))
            metrics['wasted'].append(numpy.sum(numpy.all(image_coverage == [0, 0, 255], axis=2)))
        
        #print 'Total:', 480*640
        #print 'Covered:', numpy.sum(numpy.all(image_coverage == [255, 0, 255], axis=2))
        #print 'Uncovered:', numpy.sum(numpy.all(image_coverage == [255, 0, 0], axis=2))
        #print 'Off-Target:', numpy.sum(numpy.all(image_coverage == [0, 0, 255], axis=2))
        #cv2.imshow('Filter coverage', image_coverage)
        #cv2.waitKey(10)
    print '...to', len(metrics['time'])

    return metrics


if __name__ == "__main__":

    file_ar = 'ar_tag_corners_trial_00.pickle'

    files_filter = ['filter_corners_cl_0.0_trial_00.pickle',
                    'filter_corners_cl_0.1_trial_00.pickle',
                    #'filter_corners_cl_0.25_trial_00.pickle',
                    'filter_corners_cl_0.5_trial_00.pickle',
                    'filter_corners_cl_1.0_trial_00.pickle',
                    #'filter_corners_cl_1.5_trial_00.pickle',
                    'filter_corners_cl_2.0_trial_00.pickle',
                    #'filter_corners_cl_2.5_trial_00.pickle',
                    #'filter_corners_cl_3.0_trial_00.pickle',
                    #'filter_corners_cl_3.5_trial_00.pickle',
                    #'filter_corners_cl_4.0_trial_00.pickle',
                    #'filter_corners_cl_4.5_trial_00.pickle',
                    #'filter_corners_cl_5.0_trial_00.pickle'
                    ]

    markers = '.os^*'
    for file_filter, marker in zip(files_filter, markers):
        print 'Analysing this file: ', file_filter
        metrics = unpack_pickle(file_ar, file_filter)
        # Calculate precision & recall
        tp = numpy.array(metrics['covered']).astype('float')
        fp = numpy.array(metrics['wasted']).astype('float')
        fn = numpy.array(metrics['uncovered']).astype('float')
        precision = tp / (tp + fp)  # how much of the filter is over the object?
        recall = tp / (tp + fn)  # how much of the object is covered?
        pyplot.plot(recall[-1], precision[-1], marker+'-', label=file_filter[15:21])
        print

    pyplot.axis([0, 1, 0, 1])
    pyplot.legend(loc='best')
    pyplot.xlabel('Recall')
    pyplot.ylabel('Precision')
    pyplot.title('Precision-Recall Curve for Probabilistic Privacy Filter')
    pyplot.show()
