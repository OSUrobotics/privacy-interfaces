#!/usr/bin/env python

# Pretty much all of this was borrowed from Dan Lazewatsky's package robot_vector_control. 
# Modifications by Matt Rueben starting 30-June-2014.

import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import Marker, InteractiveMarkerControl
from geometry_msgs.msg import PoseStamped
import tf
import sys

translation = None
rotation = None

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.1
    marker.scale.y = msg.scale * 0.1
    marker.scale.z = msg.scale * 0.1
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def make6DofMarker(frame_id, fixed=False, description="Simple 6-DOF Control"):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.scale = 1

    int_marker.name = "simple_6dof"
    int_marker.description = description

    # insert a box
    makeBoxControl(int_marker)

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 1
    control.orientation.y = 0
    control.orientation.z = 0
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 1
    control.orientation.z = 0
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = 1
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = 1
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    if fixed:
        control.orientation_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

    return int_marker

def processFeedback(feedback):
    global translation
    global rotation
    p = feedback.pose.position
    r = feedback.pose.orientation
    translation = p.x, p.y, p.z
    rotation = r.x, r.y, r.z, r.w

if __name__=="__main__":
    if len(sys.argv) >= 3:
        rospy.init_node("gaze_control")
        tfb = tf.TransformBroadcaster()

        # create an interactive marker server on the topic namespace simple_marker
        server = InteractiveMarkerServer("simple_marker")
        parent_frame_id = sys.argv[1]
        child_frame_id = sys.argv[2]
        server.insert(make6DofMarker(parent_frame_id, fixed=False), processFeedback)

        # 'commit' changes and send to all clients
        server.applyChanges()

        tf_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if translation and rotation:
                stamp = rospy.Time.now()
                tfb.sendTransform(translation, rotation, stamp, child_frame_id, parent_frame_id)
            tf_rate.sleep()
    else:
        print 'Usage: interactive_gaze_control.py [parent frame id] [child frame id]'
