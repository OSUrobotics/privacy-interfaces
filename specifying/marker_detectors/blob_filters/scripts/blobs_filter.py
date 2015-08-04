#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cmvision.msg import Blobs
from cmvision_3d.msg import Blobs3d
from cv_bridge import CvBridge
import message_filters
import cv2
import copy


class Bounds():
    def __init__(self, lower, upper):
        self.lower = lower
        self.upper = upper
    def restrict(self, restrictor):
        """ Input 'restrictor' must be another Bounds object. """
        self.lower = max([self.lower, restrictor.lower])
        self.upper = min([self.upper, restrictor.upper])
    def __repr__(self):
        return str((self.lower, self.upper))


# Blob size range (min, max) for each marker type. 
# Type '_ALL' covers all other types.
# The most restrictive requirements stick!
inf = float('inf')  # shorthand
size_filter = {'_ALL': Bounds(100, inf),
               'OrangeCone': Bounds(0, inf),
               'LimeGreenNote': Bounds(0, inf),
               'PinkWand': Bounds(0, inf)}

count_filter = {'_ALL': 10,
               'OrangeCone': inf,
               'LimeGreenNote': 2,
               'PinkWand': 1}

publishers_2d = {'_ALL': rospy.Publisher('/blobs/filtered/all', Blobs),
               'OrangeCone': rospy.Publisher('/blobs/filtered/orange_cone', Blobs),
               'LimeGreenNote': rospy.Publisher('/blobs/filtered/lime_green_note', Blobs),
               'PinkWand': rospy.Publisher('/blobs/filtered/pink_wand', Blobs)}

publishers_3d = {'_ALL': rospy.Publisher('/blobs_3d/filtered/all', Blobs3d),
               'OrangeCone': rospy.Publisher('/blobs_3d/filtered/orange_cone', Blobs3d),
               'LimeGreenNote': rospy.Publisher('/blobs_3d/filtered/lime_green_note', Blobs3d),
               'PinkWand': rospy.Publisher('/blobs_3d/filtered/pink_wand', Blobs3d)}


def collapse_filters():
    for key in size_filter:
        if key != '_ALL':
            size_filter[key].restrict(size_filter['_ALL'])
    print 'Sizes: ', size_filter

    for key in count_filter:
        if key != '_ALL':
            count_filter[key] = min(count_filter[key], count_filter['_ALL'])
    print 'Counts:', count_filter


def is_good_blob(blob):
    size_bounds = size_filter[blob.name]
    return (blob.area >= size_bounds.lower) & (blob.area <= size_bounds.upper)

    
def filter_blobs_by_size(blobs):
    blobs.blobs = filter(is_good_blob, blobs.blobs)
    return blobs


def filter_blobs_by_count(blobs):
    blobs_dict = {}
    for blob in blobs.blobs:
        if blob.name not in blobs_dict:
            blobs_dict[blob.name] = copy.deepcopy(blobs)
            blobs_dict[blob.name].blobs = []
            blobs_dict[blob.name].blob_count = 0
        blobs_dict[blob.name].blobs.append(blob)
        blobs_dict[blob.name].blob_count += 1

    for key in blobs_dict:
        blobs_dict[key].blobs.sort(reverse = True, 
                                   key = lambda blob: blob.area)  # sort by largest area first
        if count_filter[key] != float('inf'):
            blobs_dict[key].blobs = blobs_dict[key].blobs[0:count_filter[key]]  # take largest X areas

    # Reconstruct a msg of all the Blobs (within the count limits)
    if blobs_dict:  # if non-empty dictionary of blobs
        blobs_all = []
        for key, msg in blobs_dict.iteritems():
            blobs_all += msg.blobs
        blobs_dict['_ALL'] = copy.deepcopy(blobs)
        blobs_dict['_ALL'].blobs = blobs_all
        blobs_dict['_ALL'].blob_count = len(blobs_dict['_ALL'].blobs)

    return blobs_dict


blobs_to_show = []
        
def blobs_callback(blobs_2d, blobs_3d):
    # 2d
    blobs_goldilox_2d = filter_blobs_by_size(blobs_2d)
    blobs_dict_2d = filter_blobs_by_count(blobs_goldilox_2d)

    for key in publishers_2d:  # for each name (and '_ALL')...
        if key in blobs_dict_2d:  # ...if we got any blobs by that name...
            publishers_2d[key].publish(blobs_dict_2d[key])  # ...publish them!

    # 3d (yes, this is doing the same work again)
    blobs_goldilox_3d = filter_blobs_by_size(blobs_3d)
    blobs_dict_3d = filter_blobs_by_count(blobs_goldilox_3d)

    for key in publishers_3d:  # for each name (and '_ALL')...
        if key in blobs_dict_3d:  # ...if we got any blobs by that name...
            publishers_3d[key].publish(blobs_dict_3d[key])  # ...publish them!

    if debug_on:
        global blobs_to_show
        if '_ALL' in blobs_dict_2d:
            blobs_to_show = blobs_dict_2d['_ALL']
        else:
            blobs_to_show = []


bridge = CvBridge()

def show_blobs(image):
    if debug_on:
        image_cv = bridge.imgmsg_to_cv2(image)
        if blobs_to_show:
            for blob in blobs_to_show.blobs:
                cv2.rectangle(image_cv, (blob.left, blob.top), (blob.right, blob.bottom), 
                              (blob.blue, blob.green, blob.red))
        cv2.imshow('Image with Filtered Blobs', image_cv)
        cv2.waitKey(1)


debug_on = False


if __name__ == '__main__':
    rospy.init_node('blobs_filter')
    debug_on = rospy.get_param('blobs_filter/debug_on', False)
    
    collapse_filters()

    sub_2d = message_filters.Subscriber('/blobs', Blobs)
    sub_3d = message_filters.Subscriber('/blobs_3d', Blobs3d)
    time_syncer = message_filters.TimeSynchronizer([sub_2d, sub_3d], 10)
    time_syncer.registerCallback(blobs_callback)

    if debug_on:
        rospy.Subscriber('/camera/rgb/image_rect_color', Image, show_blobs)
    rospy.spin()
