#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped


class Republisher():
    def __init__(self):

        self.need_first_pose_array = True
        rospy.Subscriber('/particlecloud', PoseArray, self.pose_array_callback)

        self.need_first_pose = True
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        while self.need_first_pose_array or self.need_first_pose:
            rospy.sleep(0.01)

        self.pub = rospy.Publisher('/particlecloud_republish', PoseArray)
        self.pub_pose = rospy.Publisher('/amcl_pose_republish', PoseWithCovarianceStamped)

    def pose_array_callback(self, pose_array):
        self.pose_array = pose_array
        rospy.loginfo('Got a PoseArray from AMCL!')
        if self.need_first_pose_array:
            self.need_first_pose_array = False

    def pose_callback(self, pose):
        self.pose = pose
        rospy.loginfo('Got a PoseWithCovarianceStamped from AMCL!')
        if self.need_first_pose:
            self.need_first_pose = False

    def run(self):
        rospy.loginfo('Republishing pose array...now!')
        
        r = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.pub.publish(self.pose_array)
            self.pub_pose.publish(self.pose)
            r.sleep()
        
if __name__ == "__main__":

    rospy.init_node('republish_amcl')
    republisher = Republisher()
    republisher.run()
