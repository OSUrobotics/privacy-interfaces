#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray


class Republisher():
    def __init__(self):

        self.need_first_pose_array = True
        rospy.Subscriber('/particlecloud', PoseArray, self.pose_array_callback)

        while self.need_first_pose_array:
            rospy.sleep(0.01)

        self.pub = rospy.Publisher('/particlecloud_republish', PoseArray)

    def pose_array_callback(self, pose_array):
        self.pose_array = pose_array
        rospy.loginfo('Got a PoseArray from AMCL!')
        if self.need_first_pose_array:
            self.need_first_pose_array = False

    def run(self):
        rospy.loginfo('Republishing pose array...now!')
        
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.pub.publish(self.pose_array)
            r.sleep()
        
if __name__ == "__main__":

    rospy.init_node('republish_amcl')
    republisher = Republisher()
    republisher.run()
