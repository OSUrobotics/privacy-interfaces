#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped
import tf

from zone_server.msg import PrivacyZone
from zone_server.srv import *

from tag_trackers.tag_tools import Tag  # defines Tag() class, defines tf frames for 3d point tags


class MouseClickTracker():
    def __init__(self):
        self.tags = []  # blob *centers* are used as tag positions
        self.next_name = 0

        # If it's within this distance, we say it's the same tag
        self.threshold = 0.10  # 10 cm; ~4 inches

        # Wait for zone services
        rospy.wait_for_service('add_privacy_zone')
        rospy.wait_for_service('delete_privacy_zone')

        # Define zone services
        self.add_privacy_zone = rospy.ServiceProxy('add_privacy_zone', AddPrivacyZone)
        self.delete_privacy_zone = rospy.ServiceProxy('delete_privacy_zone', DeletePrivacyZone)

        self.broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('/clicked_point', PointStamped, self.callback_click)

        rospy.Timer(rospy.Duration(0.1), self.broadcast_tag_frames)

    def broadcast_tag_frames(self, event):
        for tag in self.tags:
            tag.broadcast_tf_frame(self.broadcaster)

    def callback_click(self, clicked):
        rospy.loginfo('CLICK_TRACKER> Got a clicked point in three dimensions!')
        # Associate (NEWEST!) blob with existing tag, if possible
        is_new_tag = True
        for tag in self.tags:
            if (tag - clicked.point) <= self.threshold:  # if it matches an existing tag, REMOVE that tag. 
                resp = self.delete_privacy_zone(tag.zone.name)  # call to server
                self.tags.remove(tag)
                is_new_tag = False
                break  # in case it's close to *two* tags, just update the first one. WARNING: tag names could get switched. 
        if is_new_tag:  # if it doesn't match any existing tags, add it.
            self.tags.append(Tag('clicked_' + str(self.next_name), 
                                 clicked.point, 
                                 clicked.header, 
                                 'mouse'))  
            self.next_name += 1
            try:
                resp = self.add_privacy_zone(self.tags[-1].to_zone())  # call to server
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('track_mouse_clicks')
    tracker = MouseClickTracker()
    rospy.spin()
    rospy.loginfo('Mouse click tracker has gone down! All done. ')
    

