#!/usr/bin/env python

import rospy
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from person_filter.msg import SkeletonArray


class ConverterToUV():
    def __init__(self, info_topic):
        self.camera_model = PinholeCameraModel()
        rospy.Subscriber(info_topic, CameraInfo, self.update_model)

    def update_model(self, info):
        self.camera_model.fromCameraInfo(info)
        rospy.loginfo('Camera model updated!')


class SkeletonsToUV(ConverterToUV):
    def __init__(self, skeletons_topic, info_topic):
        ConverterToUV.__init__(self, info_topic)
        self.pub = rospy.Publisher(skeletons_topic + '_uv', SkeletonArray)
        rospy.Subscriber(skeletons_topic, SkeletonArray, self.skeletons_callback)

    def skeletons_callback(self, skeleton_array):
        """ Convert skeletons to UV. """
        for s in range(len(skeleton_array.skeletons)):
            for j in range(len(skeleton_array.skeletons[s].joints)):
                skeleton_array.skeletons[s].joints[j].uv = self.camera_model.project3dToPixel(
                    skeleton_array.skeletons[s].joints[j].xyz)
        self.pub.publish(skeleton_array)
        rospy.loginfo('Skeletons (re)published!')


if __name__ == "__main__":
    rospy.init_node('skeletons_to_UV')
    skeletons_to_uv = SkeletonsToUV('/skeletons', '/camera/rgb/camera_info')
    
    rospy.spin()
    
