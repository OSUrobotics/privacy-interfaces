#!/usr/bin/env python

import rospy
from cmvision_3d.msg import Blobs3d
from geometry_msgs.msg import Point  # this defn will be modified below!
from marker_server.msg import PrivacyZone

def __sub__(self, other):
    difference = Point()
    difference.x = self.x - other.x
    difference.y = self.y - other.y
    difference.z = self.z - other.z
    return difference

Point.__sub__ = __sub__

class PhysicalMarkerTracker():
    def __init__(self):
        self.tags = []
        rospy.Subscriber('/blobs_3d/filtered/lime_green_note', Blobs3d, self.callback_blobs_3d)

    def callback_blobs_3d(self, blobs):
        for blob in blobs.blobs:
            for tag in self.tags:
                tag['center'] - blob
        self.tags['lime_green_note'].append((blobs.header, blobs.blobs[0]))
        # ADD MARKER TO SERVER

        # REMOVE MARKER FROM SERVER

if __name__ == '__main__':
    rospy.init_node('track_physical_markers')
    
