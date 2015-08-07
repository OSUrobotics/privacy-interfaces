#!/usr/bin/env python

import rospy
from zone_server.msg import PrivacyZone, PrivacyZones
from zone_server.srv import *

class PrivacyZoneServer():
    def __init__(self):
        self.zone_list = PrivacyZones()
        self.publisher = rospy.Publisher('/privacy_zones', PrivacyZones)

    def update_zone_times(self, event):
        """ Check for expired zones; turn them off. Update zone times. """
        now = rospy.Time.now()
        self.zone_list.header.stamp = now
        for index, zone in enumerate(self.zone_list.zones):
            if now > zone.time_begin and zone.status == PrivacyZone.NOT_YET:
                rospy.loginfo('ZONE_SERVER> Zone coming online now.')
                self.zone_list.zones[index].status = PrivacyZone.ACTIVE_NOW
            if now > zone.time_end and zone.status in [PrivacyZone.ACTIVE_NOW, PrivacyZone.SWITCHED_OFF]:
                rospy.loginfo('ZONE_SERVER> Zone has expired.')
                self.zone_list.zones[index].status = PrivacyZone.EXPIRED

    def publish_zones(self, event):
        """ Publish all current, updated zones as a single message. """
        self.publisher.publish(self.zone_list)

    ###### Service functions ###########
        
    def add_zone(self, req):
        try:
            self.zone_list.zones.append(req.zone)
            rospy.loginfo('ZONE_SERVER> Zone added!')
            return AddPrivacyZoneResponse(True)  # success!
        except:
            return AddPrivacyZoneResponse(False)  # failure...unknown cause

    def edit_zone(self, req):
        for index, zone in enumerate(self.zone_list.zones):
            if zone.name == req.zone.name:  # if it's the right zone...
                self.zone_list.zones[index] = req.zone  # ...then replace it!
                return EditPrivacyZoneResponse(True)  # success!
        rospy.loginfo('ZONE_SERVER> Tried to edit zone, but no zone by that name was found!')
        return EditPrivacyZoneResponse(False)  # failure...

    def delete_zone(self, req):
        for zone in self.zone_list.zones:
            if zone.name == req.name:  # if it's the right zone...
                self.zone_list.zones.remove(zone)  # ...remove it! 
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
    
    rospy.Timer(rospy.Duration(1.0), zone_server.update_zone_times)  # check times at 1Hz
    rospy.Timer(rospy.Duration(1.0), zone_server.publish_zones)  # publish zones at 1Hz

    rospy.spin()
    
