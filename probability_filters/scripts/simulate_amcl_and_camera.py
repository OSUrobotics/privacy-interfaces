#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray, Point32, PolygonStamped
import random
import tf
from tf.transformations import *
import rosbag
from math import pi


class SimulatedRobot():
    def __init__(self):
        # Import fake camera image
        bridge = CvBridge()
        image_array = cv2.imread('/home/ruebenm/workspaces/privacy_ws/src/probability_filters/pics/old-men.jpg')
        self.image_msg = bridge.cv2_to_imgmsg(image_array, "bgr8")

        # Import fake camera info
        bag = rosbag.Bag('/home/ruebenm/workspaces/privacy_ws/src/probability_filters/bags/asus_xtion_camera_info.bag', 'r')
        infos = bag.read_messages(topics='/camera/rgb/camera_info')
        self.camera_info = infos.next()[1]

        # Init AMCL stuff
        self.devs = [dev * 0.0005 for dev in range(1000, 100, -3)]
        self.N = range(110, 10, -1)*3
        self.N.sort(reverse=True)

        # Init publishers and broadcasters
        self.pub_image = rospy.Publisher('/camera/rgb/image_color', Image)
        self.pub_camera_info = rospy.Publisher('/camera/rgb/camera_info', CameraInfo)
        self.pub_amcl = rospy.Publisher('/particlecloud', PoseArray)
        self.br = tf.TransformBroadcaster()
        self.pub_polygon = rospy.Publisher('/object_bounds', PolygonStamped)

    def run(self):
        """ Continuously send out information, as if the robot is running. """
        rospy.loginfo('Running robot simulator...now!')
        r = rospy.Rate(30)
        i = 0
        while not rospy.is_shutdown():
            self.update_amcl(self.devs[i], self.N[i])
            #self.update_amcl(0.01, 10)
            self.update_polygon(0.15)
            self.send()

            if i == len(self.devs) - 1:
                i = 0
            else:
                i += 1

            r.sleep()

    def send(self):
        """ Broadcast TF frames and Publish stuff. """
        self.pub_image.publish(self.image_msg)
        self.pub_camera_info.publish(self.camera_info)
        self.pub_amcl.publish(self.amcl)
        self.br.sendTransform((4.0, 3.0, 0.6),
                              (0.0, 0.0, 0.0, 1.0),
                              rospy.Time.now(),
                              '/private_object',
                              '/map')
        self.br.sendTransform((2.0, 3.0, 0.5),
                              tuple(quaternion_multiply(quaternion_about_axis(pi/2, (0,1,0)),
                                        quaternion_about_axis(-pi/2, (0,0,1)))),
                              rospy.Time.now(),
                              '/camera_rgb_optical_frame',
                              '/map')
        self.pub_polygon.publish(self.polygon)

    def update_amcl(self, dev, N):
        """ Choose new AMCL pose array. """
        self.amcl = PoseArray()  # erase old poses
        self.amcl.header.frame_id = "/map"
        self.amcl.header.stamp = rospy.Time.now()
        for n in range(N):
            pose = Pose()
            pose.position.x = random.gauss(2.0, dev)
            pose.position.y = random.gauss(3.0, dev)
            pose.position.z = 0.0

            z_axis = (0, 0, 1)
            angle = random.gauss(0.0, dev)
            q = quaternion_about_axis(angle, z_axis)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            self.amcl.poses.append(pose)

    def update_polygon(self, s):
        self.polygon = PolygonStamped()  # erase old polygon
        self.polygon.header.frame_id = '/private_object'
        self.polygon.header.stamp = rospy.Time.now()
        for i in range(4):  # add four vertices
            self.polygon.polygon.points.append(Point32())
        r = s / 2.0  # radius = 1/2 * side length
        self.polygon.polygon.points[0].y = -1 * r
        self.polygon.polygon.points[0].z = -1 * r
        self.polygon.polygon.points[1].y =      r
        self.polygon.polygon.points[1].z = -1 * r
        self.polygon.polygon.points[2].y =      r
        self.polygon.polygon.points[2].z =      r
        self.polygon.polygon.points[3].y = -1 * r
        self.polygon.polygon.points[3].z =      r
        

if __name__ == "__main__":

    rospy.init_node('simulate_amcl_and_camera')
    robot = SimulatedRobot()
    robot.run()
