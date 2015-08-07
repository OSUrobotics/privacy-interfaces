import rospy
from zone_server.msg import PrivacyZone
from shape_msgs.msg import SolidPrimitive
from math import sqrt

class Tag():
    def __init__(self, name, center, header, tagging_mode):
        self.name = name
        self.center = center
        self.header = header
        self.tagging_mode = tagging_mode  # e.g., 'marker', 'wand', 'gui'
        self.zone = self.to_zone()

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

    def broadcast_tf_frame(self, broadcaster):
        """ Sends out a tf transformation to the center of the tag.
        Orientation is arbitrary (same as parent frame). """
        broadcaster.sendTransform((self.center.x, self.center.y, self.center.z),
                                  (0,0,0,1),
                                  rospy.Time.now(),
                                  self.name,
                                  self.header.frame_id)

    def to_zone(self):
        zone = PrivacyZone()
        zone.name = self.name
        zone.labels.append(self.tagging_mode)
        zone.status = PrivacyZone.ACTIVE_NOW
        zone.time_begin = rospy.Time(0)  # active since the beginning of time
        zone.time_end = rospy.Time.now() + rospy.Duration(60*60*24)  # active for one day
        zone.header = self.header
        zone.extents.type = SolidPrimitive.BOX
        zone.extents.dimensions = [0.20, 0.20, 0.20]  # a 20 cm (~8 in) cube, centered around its tf frame
        
        return zone
