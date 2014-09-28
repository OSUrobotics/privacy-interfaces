#!/usr/bin/env python

import rospy
import tf
import numpy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import PointStamped
from math import sqrt


class StandingStillMeter():
    def __init__(self, transition_time, stillness_radius):
        """ A person is standing still if he or she remains within a
        certain radius (INPUT #2, in meters) for a certain amount of
        time (INPUT #1, in seconds). """
        self.transition_time = transition_time
        self.stillness_radius = stillness_radius
        
        self.lis = tf.TransformListener()
        
        self.is_still = False
        self.torso_history = []  # center of mass position buffer
        self.time_history = []  # ...and associated time values (approximate)
        self.have_sufficient_torso_history = False

        self.pub_stillness = rospy.Publisher('/is_standing_still', Bool)
        self.pub_torso = rospy.Publisher('/torso_location', PointStamped)

    def get_skeleton(self):
        self.skelly = {'head_1': (),
                       'neck_1': (),
                       'left_shoulder_1': (),
                       'left_elbow_1': (),
                       'left_hand_1': (),
                       'right_shoulder_1': (),
                       'right_elbow_1': (),
                       'right_hand_1': (),
                       'torso_1': (),
                       'left_hip_1': (),
                       'left_knee_1': (),
                       'left_foot_1': (),
                       'right_hip_1': (),
                       'right_knee_1': (),
                       'right_foot_1': ()}
        for joint in self.skelly.keys():
            translation, rotation = self.lis.lookupTransform('/openni_rgb_optical_frame', joint, rospy.Time(0))
            self.skelly[joint] = list(translation)

    def publish_torso_msg(self):
        self.torso_msg = PointStamped()
        self.torso_msg.header.frame_id = '/openni_rgb_optical_frame'
        self.torso_msg.header.stamp = rospy.Time.now()
        [self.torso_msg.point.x,
         self.torso_msg.point.y,
         self.torso_msg.point.z] = self.skelly['torso_1']
        self.pub_torso.publish(self.torso_msg)

    def publish_whether_still(self):
        # Append newest measurement
        self.torso_history.append(self.skelly['torso_1'])
        self.time_history.append(rospy.Time.now())

        # Check newest vs. oldest
        # If distance is too big, pop the oldest and repeat
        while sqrt(sum([(new - old)**2 for old, new in zip(self.torso_history[0], self.torso_history[-1])])) > self.stillness_radius:
            self.torso_history.pop(0)
            self.time_history.pop(0)

        rospy.loginfo('Current torso history buffer size: {0}'.format(len(self.torso_history)))

        # Check if we have covered at least one transition time period
        if (self.time_history[-1] - self.time_history[0]).to_sec() > self.transition_time:
            self.is_standing_still = True
        else:
            self.is_standing_still = False

        # Publish message with result
        stillness_msg = Bool()
        stillness_msg.data = self.is_standing_still
        self.pub_stillness.publish(stillness_msg)
        
    def run(self):
        r = rospy.Rate(10)
        warned = False
        while not rospy.is_shutdown():
            if self.lis.frameExists('head_1'):
                if warned:
                    rospy.loginfo('OK, the skeleton is now in the /tf tree.')
                    warned = False

                self.get_skeleton()
                self.publish_torso_msg()

                self.publish_whether_still()
            else:
                if not warned:
                    rospy.logwarn('The skeleton you are looking for is not in the /tf tree. Try restarting the tracker!')
                    warned = True

            r.sleep()
    

if __name__ == "__main__":

    rospy.init_node('is_standing_still')
    transition_time = rospy.get_param('transition_time', 3.0)
    stillness_radius = rospy.get_param('stillness_radius', 0.5)
    meter = StandingStillMeter(transition_time,
                               stillness_radius)
    meter.run()
