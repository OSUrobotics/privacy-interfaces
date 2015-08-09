#!/usr/bin/env python

import rospy
from cmvision_3d.msg import Blobs3d
import tf

from zone_server.msg import PrivacyZone
from zone_server.srv import *

from tag_trackers.tag_tools import Tag  # defines Tag() class, defines tf frames for 3d point tags


class MagicWandTracker():
    def __init__(self):
        self.tags = []  # blob *centers* are used as tag positions
        self.next_name = 0

        # If it's within this distance, we say it's the same tag
        self.threshold = 0.10  # 10 cm; ~4 inches

        # If the wand stays within "tolerance" for "time" seconds, it counts as a "linger"
        self.blobses = []
        self.last_linger = rospy.Time(0)  # last time we lingered
        self.linger_tolerance = 0.05  # 5 cm; ~2 inches
        self.linger_time = rospy.Duration(2.0)  # 2 seconds
        self.linger_cooldown = rospy.Duration(4.0)  # 4 seconds

        # Wait for zone services
        rospy.wait_for_service('add_privacy_zone')
        rospy.wait_for_service('delete_privacy_zone')

        # Define zone services
        self.add_privacy_zone = rospy.ServiceProxy('add_privacy_zone', AddPrivacyZone)
        self.delete_privacy_zone = rospy.ServiceProxy('delete_privacy_zone', DeletePrivacyZone)

        self.broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('/blobs_3d/filtered/pink_wand', Blobs3d, self.callback_blobs_3d)

        rospy.Timer(rospy.Duration(0.1), self.broadcast_tag_frames)

    def broadcast_tag_frames(self, event):
        for tag in self.tags:
            tag.broadcast_tf_frame(self.broadcaster)

    def callback_blobs_3d(self, blobs):
        if len(blobs.blobs) > 1:
            rospy.logerr('WAND_TRACKER> For the magic wand, the blobs filter should return at most ONE. ')

        # ring buffer of wand blobs
        self.blobses.append(blobs)  # add newest one to end
        if len(self.blobses) > 1:
            while (self.blobses[-1].header.stamp - self.blobses[1].header.stamp) > self.linger_time:
                self.blobses.pop(0)  # reduce to correct time period
        print len(self.blobses)
        
        if self.is_lingering():
            rospy.loginfo('WAND_TRACKER> Wand has lingered!')
            # Associate (NEWEST!) blob with existing tag, if possible
            for blob in blobs.blobs:  # should only be one of these
                is_new_tag = True
                for tag in self.tags:
                    if (tag - blob.center) <= self.threshold:  # if it matches an existing tag, REMOVE that tag. 
                        resp = self.delete_privacy_zone(tag.zone.name)  # call to server
                        self.tags.remove(tag)
                        is_new_tag = False
                        break  # in case it's close to *two* tags, just update the first one. WARNING: tag names could get switched. 
                if is_new_tag:  # if it doesn't match any existing tags, add it.
                    self.tags.append(Tag('wand_' + str(self.next_name), 
                                         blob.center, 
                                         blobs.header, 
                                         'wand'))  
                    self.next_name += 1
                    try:
                        resp = self.add_privacy_zone(self.tags[-1].to_zone())  # call to server
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e

    def is_lingering(self):
        """ Checks if magic wand has stayed in about the same place
        for a few seconds. THERE IS WORK BEING REPEATED HERE. """
        if (self.blobses[-1].header.stamp - self.blobses[0].header.stamp) > self.linger_time:  # if we have a long enough time history
            if (rospy.Time.now() - self.last_linger) > self.linger_cooldown:  # and if we have cooled down from the last time we lingered
                min_x = min([blobs.blobs[0].center.x for blobs in self.blobses])
                max_x = max([blobs.blobs[0].center.x for blobs in self.blobses])
                
                min_y = min([blobs.blobs[0].center.y for blobs in self.blobses])
                max_y = max([blobs.blobs[0].center.y for blobs in self.blobses])
                
                min_z = min([blobs.blobs[0].center.z for blobs in self.blobses])
                max_z = max([blobs.blobs[0].center.z for blobs in self.blobses])

                if max(((max_x - min_x),
                        (max_y - min_y),
                        (max_z - min_z))) < self.linger_tolerance:
                    self.last_linger = rospy.Time.now()
                    return True
                else:
                    return False

if __name__ == '__main__':
    rospy.init_node('track_magic_wand')
    tracker = MagicWandTracker()
    rospy.spin()
    rospy.loginfo('Magic wand tracker has gone down! All done. ')
    

