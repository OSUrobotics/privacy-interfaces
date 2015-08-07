#!/usr/bin/env python

import rospy
from zone_server.msg import PrivacyZone, PrivacyZones
from zone_server.srv import *

class PrivacyZoneServer():
    def __init__(self):
        self.zones = PrivacyZones()
        self.publisher = rospy.Publisher('/privacy_zones', PrivacyZones)

    def update_zone_times(self):
        """ Check for expired zones; turn them off. Update zone times. """
        self.zones.header.stamp = rospy.Time.now()

    def broadcast_tf_frames(self):
        """ Broadcast the tf frame for each zone. """
        pass

    def publish_zones(self):
        """ Publish all current, updated zones as a single message. """
        self.publisher.publish(self.zones)

    ###### Service functions ###########
        
    def add_zone(self, req):
        try:
            self.zones.append(req.zone)
            rospy.loginfo('ZONE_SERVER> Zone added!')
            return AddPrivacyZoneResponse(True)  # success!
        except:
            return AddPrivacyZoneResponse(False)  # failure...unknown cause

    def edit_zone(self, req):
        for zone in self.zones:
            if zone.name == req.zone.name:  # if it's the right zone...
                zone = req.zone             # ...then replace it!
                return EditPrivacyZoneResponse(True)  # success!
        rospy.loginfo('ZONE_SERVER> Tried to edit zone, but no zone by that name was found!')
        return EditPrivacyZoneResponse(False)  # failure...

    def delete_zone(self, req):
        for zone in self.zones:
            if zone.name == req.name:  # if it's the right zone...
                self.zones.remove(zone)  # ...remove it! 
                return DeletePrivacyZoneResponse(True)  # success!
        rospy.loginfo('ZONE_SERVER> Tried to delete zone, but no zone by that name was found!')
        return DeletePrivacyZoneResponse(False)  # failure...
        
    #######################################


if __name__ == "__main__":
    rospy.init_node('zone_server')
    
    zone_server = PrivacyZoneServer()

    # Services for handling PrivacyZones
    srv_add = rospy.Service('add_privacy_zone', AddPrivacyZone, zone_server.add_zone)
    srv_edit = rospy.Service('edit_privacy_zone', EditPrivacyZone, zone_server.edit_zone)
    srv_delete = rospy.Service('delete_privacy_zone', DeletePrivacyZone, zone_server.delete_zone)
    
    pacemaker = rospy.Rate(10.0)  # 10Hz
    while not rospy.is_shutdown():
        zone_server.update_zone_times()
        zone_server.broadcast_tf_frames()
        zone_server.publish_zones()
    
