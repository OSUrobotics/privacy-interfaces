#!/usr/bin/env python
# 

import rospy
import os
import sys
import rosbag
from sensor_msgs.msg import Image
import numpy
import cv2
from cv_bridge import CvBridge
from matplotlib import pyplot


class FilterEvaluator():
    def __init__(self):
        self.bridge = CvBridge()
        self.metrics = {'t': [],
                        'coverage': [],
                        'waste': [],
                        'face_size': []}
        self.first_frame = True

    def open_bags(self, path_bag_video, path_bag_test, path_bag_control):
        self.bag_video = rosbag.Bag(path_bag_video)
        self.video_frames = self.bag_video.read_messages(topics='/camera/rgb/image_color')

        self.bag_test = rosbag.Bag(path_bag_test)
        self.test_images = self.bag_test.read_messages(topics='image')
        self.test_masks = self.bag_test.read_messages(topics='mask')

        self.bag_control = rosbag.Bag(path_bag_control)
        self.control_images = self.bag_control.read_messages(topics='image')
        self.control_masks = self.bag_control.read_messages(topics='mask')

        self.msg_count = 0
        
    def get_next_messages(self):
        print 'Incrementing!'
        # Handle time
        video_frame = self.video_frames.next()[1]
        if self.first_frame:
            self.t = 0.0
            self.t_ref = video_frame.header.stamp.to_sec()
            self.first_frame = False
        else:
            self.t = video_frame.header.stamp.to_sec() - self.t_ref

        # Test image
        test_image = self.test_images.next()[1]
        self.test_image = self.bridge.imgmsg_to_cv2(test_image)

        # Test mask
        test_mask = self.test_masks.next()[1]
        self.test_mask = self.bridge.imgmsg_to_cv2(test_mask)

        # Control image
        control_image = self.control_images.next()[1]
        self.control_image = self.bridge.imgmsg_to_cv2(control_image)

        # Control mask
        control_mask = self.control_masks.next()[1]
        self.control_mask = self.bridge.imgmsg_to_cv2(control_mask)

        self.msg_count += 1

    def compare_masks(self):
        # Metrics:
        #  1. Face pixels covered
        #  2. Non-face pixels covered
        #  3. Total face size
        
        image = self.control_image.copy()
        face_control = image[self.control_mask > 0]
        face_test = image[self.test_mask > 0]
        coverage = image[numpy.logical_and(self.control_mask > 0,
                                                self.test_mask > 0)]
        
        self.metrics['t'].append(self.t)
        self.metrics['coverage'].append(float(coverage.size) / image.size)
        self.metrics['waste'].append(float(face_test.size - coverage.size) / image.size)
        self.metrics['face_size'].append(float(face_control.size) / image.size)

    def display_masks(self):
        image = self.control_image.copy()

        # Magic so we can see where the masks overlap
        image /= 9
        image[self.test_mask > 0] *= 3
        image[self.control_mask > 0] *= 3

        cv2.putText(image, 
                    'Time: {3}s, Coverage: {0}%, Waste: {1}%, Face Size: {2}%'.format(int(self.metrics['coverage'][-1]*1000)/10.0, 
                                                                                      int(self.metrics['waste'][-1]*1000)/10.0, 
                                                                                      int(self.metrics['face_size'][-1]*1000)/10.0, 
                                                                                      int(self.metrics['t'][-1]*1000)/1000.0),
                    (20, 480-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))
        cv2.imshow('Masks', cv2.pyrUp(image))
        cv2.waitKey(1000/5)

    def close_bags(self):
        self.bag_test.close()
        self.bag_control.close()

    def plot_metrics(self):
        pyplot.plot(self.metrics['t'], self.metrics['face_size'], 'go', label='Face Size')
        pyplot.plot(self.metrics['t'], self.metrics['coverage'], 'bo', label='Face Coverage')
        pyplot.plot(self.metrics['t'], self.metrics['waste'], 'ro', label='Non-Face Coverage')
        pyplot.title('Face Tracking Results')
        pyplot.xlabel('Time (s)')
        pyplot.ylabel('Fraction of Total Image Size')
        pyplot.legend(loc='right')
        pyplot.show()


def evaluate_filter():
    """ Main function """
    rospy.init_node('evaluate_filter')
    
    # Handle arguments
    if len(sys.argv) < 2:
        rospy.logerr('Must pass at least the first argument: the VIDEO bag file path.')
        return -1
    if len(sys.argv) < 3:
        rospy.logerr('Must pass at least the second argument: the TEST bag file path.')
        return -1
    if len(sys.argv) < 4:
        rospy.logerr('Must pass at least the first argument: the CONTROL bag file path.')
        return -1
    else:
        path_bag_video = sys.argv[1]
        path_bag_test = sys.argv[2] 
        path_bag_control = sys.argv[3] 

    filter_evaluator = FilterEvaluator()
    filter_evaluator.open_bags(path_bag_video, path_bag_test, path_bag_control)

    while not rospy.is_shutdown():
        # Get next image
        try:  
            filter_evaluator.get_next_messages()
            filter_evaluator.compare_masks()
            filter_evaluator.display_masks()
        except StopIteration:
            rospy.loginfo('Processed last image (#{0}); exiting!'.format(filter_evaluator.msg_count))
            filter_evaluator.close_bags()
            filter_evaluator.plot_metrics()
            return 0
        

if __name__ == "__main__":

    evaluate_filter()
    
    cv2.destroyAllWindows()
