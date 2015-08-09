#!/usr/bin/env python

import rospy
from cmvision_3d.msg import Blobs3d
from geometry_msgs.msg import Point  # this defn will be modified below!
import tf

from zone_server.msg import PrivacyZone
from zone_server.srv import *

from tag_trackers.tag_tools import Tag  # defines Tag() class, defines tf frames for 3d point tags


class PhysicalMarkerTracker():
    def __init__(self):
        self.tags = []  # blob *centers* are used as tag positions
        self.next_name = 0

        # If it's within this distance, we say it's the same tag
        self.threshold = 0.05  # 5 cm; ~2 inches

        # If it hasn't been seen for this long, we say it's been removed
        self.timeout = rospy.Duration(3.0)  # 3 seconds

        # Wait for zone services
        rospy.wait_for_service('add_privacy_zone')
        rospy.wait_for_service('edit_privacy_zone')
        rospy.wait_for_service('delete_privacy_zone')

        # Define zone services
        self.add_privacy_zone = rospy.ServiceProxy('add_privacy_zone', AddPrivacyZone)
        self.edit_privacy_zone = rospy.ServiceProxy('edit_privacy_zone', EditPrivacyZone)
        self.delete_privacy_zone = rospy.ServiceProxy('delete_privacy_zone', DeletePrivacyZone)

        self.broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('/blobs_3d/filtered/lime_green_note', Blobs3d, self.callback_blobs_3d)
        rospy.Timer(rospy.Duration(1.0), self.check_for_stale_tags)
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tag_frames)

    def check_for_stale_tags(self, event):
        stale_ones = [tag for tag in self.tags if tag.get_staleness() > self.timeout]  # list of stale tags
        for stale_one in stale_ones:
            stale_one.zone.status = PrivacyZone.SWITCHED_OFF
            rospy.loginfo('TRACKER> A physical marker has gone stale!')
            resp = self.edit_privacy_zone(stale_one.zone)  # call to server
            self.tags.remove(stale_one)  # assumes no copies

    def broadcast_tag_frames(self, event):
        for tag in self.tags:
            tag.broadcast_tf_frame(self.broadcaster)

    def callback_blobs_3d(self, blobs):
        # Associate blobs with tags
        for blob in blobs.blobs:
            is_new_tag = True
            for tag in self.tags:
                if (tag - blob.center) <= self.threshold:  # if it matches an existing tag, update that tag. 
                    tag.update_position(blob.center)
                    tag.update_timestamp(blobs.header.stamp)
                    resp = self.edit_privacy_zone(tag.zone)  # call to server
                    is_new_tag = False
                    break  # in case it's close to *two* tags, just update the first one. WARNING: tag names could get switched. 
            if is_new_tag:  # if it doesn't match any existing tags, add it.
                self.tags.append(Tag('note_' + str(self.next_name), 
                                     blob.center, 
                                     blobs.header, 
                                     'marker'))  
                self.next_name += 1
                try:
                    resp = self.add_privacy_zone(self.tags[-1].to_zone())  # call to server
                except rospy.ServiceException, e:
                    print "Service call failed: %s"%e



if __name__ == '__main__':
    rospy.init_node('track_physical_markers')
    tracker = PhysicalMarkerTracker()
    rospy.spin()
    rospy.loginfo('Physical marker tracker has gone down! All done. ')
