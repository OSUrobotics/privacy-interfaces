#!/usr/bin/env python

import rospy
from cmvision_3d.msg import Blobs3d
from geometry_msgs.msg import Point  # this defn will be modified below!
from math import sqrt
#from marker_server.msg import PrivacyZone


class Tag():
    def __init__(self, name, center, header):
        self.name = name
        self.center = center
        self.header = header

    def update_position(self, center):
        self.center = center

    def update_timestamp(self, time):
        self.header.stamp = time

    def get_staleness(self):
        return rospy.Time.now() - self.header.stamp

    def __sub__(self, centerpoint):
        distance = sqrt(sum([(self.center.x - centerpoint.x)**2,
                             (self.center.y - centerpoint.y)**2,
                             (self.center.z - centerpoint.z)**2]))
        return distance


class PhysicalMarkerTracker():
    def __init__(self):
        self.tags = []  # blob *centers* are used as tag positions
        self.next_name = 0

        # If it's within this distance, we say it's the same tag
        self.threshold = 0.05  # 5 cm; ~2 inches

        # If it hasn't been seen for this long, we say it's been removed
        self.timeout = rospy.Duration(3.0)  # 3 seconds


        rospy.Subscriber('/blobs_3d/filtered/lime_green_note', Blobs3d, self.callback_blobs_3d)
        self.monitor_stale_tags()

    def monitor_stale_tags(self):
        pacemaker = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            stale_ones = [tag for tag in self.tags if tag.get_staleness() > self.timeout]  # list of stale tags
            for stale_one in stale_ones:
                print 'SWITCH MARKER OFF'
                self.tags.remove(stale_one)  # assumes no copies
            
            pacemaker.sleep()


    def callback_blobs_3d(self, blobs):

        # Associate blobs with tags
        for blob in blobs.blobs:
            is_new_tag = True
            for tag in self.tags:
                if (tag - blob.center) <= self.threshold:  # if it matches an existing tag, update that tag. 
                    tag.update_position(blob.center)
                    tag.update_timestamp(blobs.header.stamp)
                    print 'UPDATE MARKER ON SERVER'
                    is_new_tag = False
                    break  # in case it's close to *two* tags, just update the first one. WARNING: tag names could get switched. 
            if is_new_tag:
                self.tags.append(Tag(self.next_name, blob.center, blobs.header))  # if it doesn't match any existing tags, add it.
                self.next_name += 1
                print 'ADD MARKER TO SERVER'


if __name__ == '__main__':
    rospy.init_node('track_physical_markers')
    tracker = PhysicalMarkerTracker()
    rospy.loginfo('Physical marker tracker has gone down! All done. ')
