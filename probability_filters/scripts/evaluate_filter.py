#!/usr/bin/env python
# 
# Detects face using hat position, real face size, and depth information.
# Creates mask of face pixels to be used as ground truth.
# 

import rospy
import os
import sys
import rosbag
from sensor_msgs.msg import Image
import numpy
import cv2
from cv_bridge import CvBridge


class FilterEvaluator():
    def __init__(self):
        pass

    def open_bags(self, path_bag_test, path_bag_control):
        pass

    def get_next_messages(self):
        pass

    def close_bags(self):
        pass

def evaluate_filter():
    """ Main function """
    rospy.init_node('evaluate_filter')
    
    # Handle arguments
    if len(sys.argv) < 2:
        rospy.logerr('Must pass at least the first argument: the TEST bag file path.')
        return -1
    if len(sys.argv) < 3:
        rospy.logerr('Must pass at least the second argument: the CONTROL bag file path.')
        return -1
    else:
        path_bag_test = sys.argv[1] 
        path_bag_control = sys.argv[2] 

    filter_evaluator = FilterEvaluator()
    filter_evaluator.open_bags(path_bag_test, path_bag_control)

    while True:
        # Get next image
        try:  
            filter_evaluator.get_next_messages()
        except StopIteration:
            rospy.loginfo('Processed last image; exiting!')
            return 0


    filter_evaluator.close_bags()
        



if __name__ == "__main__":

    evaluate_filter()
    
    cv2.destroyAllWindows()
