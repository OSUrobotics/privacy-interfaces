#!/usr/bin/env python

import rospy
from cmvision.msg import Blobs
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

publishers = {'_ALL': rospy.Publisher('/blobs/filtered/all', Blobs),
               'OrangeCone': rospy.Publisher('/blobs/filtered/orange_cone', Blobs),
               'LimeGreenNote': rospy.Publisher('/blobs/filtered/lime_green_note', Blobs),
               'PinkWand': rospy.Publisher('/blobs/filtered/pink_wand', Blobs)}


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
        blobs_dict[key].blobs = blobs_dict[key].blobs[0:count_filter[key]]  # take largest X areas

    # Reconstruct a msg of all the Blobs (within the count limits)
    if blobs_dict:  # if non-empty dictionary of blobs
        blobs_all = []
        for key, msg in blobs_dict.iteritems():
            blobs_all += msg.blobs
        #reduce(lambda x,y: x.blobs + y.blobs, 
        #                   blobs_dict.values())  # concatenate blobs again
        #if not isinstance(blobs_all, list):
        #    blobs_all = [blobs_all]
        blobs_dict['_ALL'] = copy.deepcopy(blobs)
        blobs_dict['_ALL'].blobs = blobs_all
        blobs_dict['_ALL'].blob_count = len(blobs_dict['_ALL'].blobs)

    return blobs_dict

        
def blobs_callback(blobs_in):
    blobs_goldilox = filter_blobs_by_size(blobs_in)
    blobs_dict = filter_blobs_by_count(blobs_goldilox)

    for key in publishers:  # for each name (and '_ALL')...
        if key in blobs_dict:  # ...if we got any blobs by that name...
            publishers[key].publish(blobs_dict[key])  # ...publish them!


if __name__ == '__main__':
    rospy.init_node('blobs_filter')
    collapse_filters()
    rospy.Subscriber('/blobs', Blobs, blobs_callback)
    rospy.spin()
