#!/usr/bin/env python

import rospy
#from fake_pkg.msg import EasyRegion
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PolygonStamped, Point32
import tf
from cv_bridge import CvBridge
import cv2
import numpy
import copy


class RayTransformer():
    def __init__(self, topic):
        self.model = PinholeCameraModel()
        rospy.Subscriber(topic, CameraInfo, self.callback_info)

    def callback_info(self, info):
        self.model.fromCameraInfo(info)

    def projectPointCloudToPixels(self, cloud):
        pixel_cloud = []
        for point in cloud.points:
            pixel = self.model.project3dToPixel((point.x, point.y, point.z))
            pixel_cloud.append(pixel)
        return pixel_cloud

class ZoneTransformer():
    def __init__(self):
        self.listener = tf.TransformListener()

    def transform_polygon(self, polygon, frame_target):
        """ Use TF Listener to transform the points in a
        PolygonStamped into a different TF frame...via a PointCloud
        msg, which is the output data type. """
        cloud = PointCloud()
        cloud.header = polygon.header
        cloud.points = polygon.polygon.points
        self.listener.waitForTransform(frame_target, cloud.header.frame_id, cloud.header.stamp, rospy.Duration(1.0))
        return self.listener.transformPointCloud(frame_target, cloud)

    def transform_polygons(self, polygons, frame_target):
        """ This is transform_polygon, but for a list of PolygonStamped msgs! """
        clouds_out = []
        for polygon in polygons:
            cloud_out = self.transform_polygon(polygon, frame_target)
            clouds_out.append(cloud_out)
        return clouds_out

class ZoneGrabber():
    def __init__(self, topic):
        print 'Be sure to run: rosrun tf static_transform_publisher 2 0 1 0 0 0 1 /base_link /zone 10'

    def get_zones(self):
        # As it would be seen in a nice, zone message
        frame_id = '/zone' 
        length = 0.2  # x
        width = 0.50  # y
        height = 0.50  # z

        # Expand to a PolygonStamped of 3d points
        self.zones = []
        zone = PolygonStamped()
        zone.header.frame_id = frame_id
        zone.header.stamp = rospy.Time.now()

        # Permute!
        point = Point32()
        for point.x in [-1*length, length]:
            for point.y in [-1*width, width]:
                for point.z in [-1*height, height]:
                    zone.polygon.points.append(copy.deepcopy(point))

        # ...we'd probably have several zones
        self.zones.append(zone)

class ZoneFilter():
    def __init__(self, topic_image, topic_info, topic_zone):
        self.ray_transformer = RayTransformer(topic_info)
        self.zone_transformer = ZoneTransformer()
        self.zone_grabber = ZoneGrabber(topic_zone)
        self.bridge = CvBridge()
        topic_image_out = topic_image + '/filtered'
        self.publisher = rospy.Publisher(topic_image_out, Image)
        rospy.Subscriber(topic_image, Image, self.callback_image)
        
    def convert_points():
        pass

    def callback_image(self, image):
        # tf zone Polygon at that stamp
        self.zone_grabber.get_zones()  # this is a HACKY HACK! zones will actually come in msgs
        self.zone_grabber.zones[0].header.stamp = image.header.stamp  # HACK for timing -- WORKS!
        clouds = self.zone_transformer.transform_polygons(self.zone_grabber.zones, 
                                                          image.header.frame_id)

        # convert to pixels
        pixel_clouds = []
        for cloud in clouds:
            pixel_cloud = self.ray_transformer.projectPointCloudToPixels(cloud)
            pixel_clouds.append(pixel_cloud)

        # convex hull
        hulls = []
        for pixel_cloud in pixel_clouds:
            pixel_cloud = numpy.array(pixel_cloud).astype('int32')
            hull = cv2.convexHull(pixel_cloud)
            hulls.append(hull)

        # For the image:
        image_array = self.bridge.imgmsg_to_cv2(image)  # convert to cv2
        """
        for pixel_cloud in pixel_clouds:
            for pixel in pixel_cloud:
                pixel = tuple(int(u) for u in pixel)
                cv2.circle(image_array, pixel, 3, (255, 255, 0))
                """

        # make a blurred copy
        image_blurred = cv2.GaussianBlur(image_array, ksize=(15,15), sigmaX=20)

        # make a mask using the convex hull(s)
        mask = numpy.zeros(image_array.shape, numpy.uint8)
        for hull in hulls:
            cv2.fillConvexPoly(mask, hull, (1,1,1))

        # blur the pixels in the mask
        image_array.flags.writeable = True
        image_array[mask.astype('bool')] = image_blurred[mask.astype('bool')]
        image_array.flags.writeable = False

        # convert back to ROS msg and publish out
        image_out = self.bridge.cv2_to_imgmsg(image_array, encoding='rgb8')
        self.publisher.publish(image_out)
        print 'Publish!'



if __name__ == '__main__':
    rospy.init_node('test_filter_zones')
    zone_filter = ZoneFilter('/wide_stereo/right/image_rect_color',
                             '/wide_stereo/right/camera_info',
                             '/zones')
    rospy.spin()
