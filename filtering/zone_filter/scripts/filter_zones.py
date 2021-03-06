#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud
from image_geometry import PinholeCameraModel
from geometry_msgs.msg import PolygonStamped, Point32
import tf
from cv_bridge import CvBridge
import cv2
import numpy
import copy

from zone_server.msg import PrivacyZones, PrivacyZone


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
        cloud.header.stamp = rospy.Time(0)  # HACK
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
        self.polygons_are_ready = False
        rospy.Subscriber(topic, PrivacyZones, self.zones_callback)
        while not self.polygons_are_ready:
            rospy.sleep(0.1)

    def zones_callback(self, zones):
        active_zones = self.get_active_zones(zones)
        self.zone_labels = [zone.labels for zone in active_zones.zones]
        self.polygons = self.zones_to_polygons(active_zones)
        if not self.polygons_are_ready:
            self.polygons_are_ready = True

    def get_active_zones(self, zones):
        active_zones = PrivacyZones()
        active_zones.header = zones.header  # make a copy, essentially
        active_zones.zones = [zone for zone in zones.zones if zone.status == PrivacyZone.ACTIVE_NOW]  # active zones only
        return active_zones

    def zones_to_polygons(self, zones):
        polygons = []
        for zone in zones.zones:
            # Expand to a PolygonStamped of 3d points
            polygon = PolygonStamped()
            polygon.header = zone.header  # copy both tf frame and timestamp

            # Permute! 
            point = Point32()
            x, y, z = tuple(dim/2.0 for dim in zone.extents.dimensions)  # sides -> half-sides
            for point.x in [-1*x, x]:
                for point.y in [-1*y, y]:
                    for point.z in [-1*z, z]:
                        polygon.polygon.points.append(copy.deepcopy(point))

            polygons.append(polygon)

        return polygons


class ZoneFilter():
    def __init__(self, topic_image, topic_info, topic_zone):
        self.ray_transformer = RayTransformer(topic_info)
        self.zone_transformer = ZoneTransformer()
        self.zone_grabber = ZoneGrabber(topic_zone)
        self.bridge = CvBridge()
        topic_image_out = topic_image + '/filtered'
        self.publisher = rospy.Publisher(topic_image_out, Image)
        rospy.Subscriber(topic_image, Image, self.callback_image)

    def callback_image(self, image):
        # tf zone Polygon at that stamp
        clouds = self.zone_transformer.transform_polygons(self.zone_grabber.polygons, 
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
        image_array = self.bridge.imgmsg_to_cv2(image, desired_encoding='rgb8')  # convert to cv2

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

        # mark zone corners
        for labels, pixel_cloud in zip(self.zone_grabber.zone_labels, pixel_clouds):
            if 'marker' in labels:
                color = (0, 255, 0)  # green for sticky notes
            elif 'wand' in labels:
                color = (255, 0, 0)  # red for magic wand
            elif 'mouse' in labels:
                color = (0, 0, 255)  # blue for clicking in the GUI
            else:
                color = (0, 0, 0)  # black otherwise
            for pixel in pixel_cloud:
                pixel = tuple(int(u) for u in pixel)
                cv2.circle(image_array, pixel, 1, color)

        # convert back to ROS msg and publish out
        image_out = self.bridge.cv2_to_imgmsg(image_array, encoding='rgb8')
        image_out.header = image.header  # so it shows up in RViz!
        self.publisher.publish(image_out)


if __name__ == '__main__':
    rospy.init_node('filter_zones')
    zone_filter = ZoneFilter('/camera/rgb/image_rect_color',
                             '/camera/rgb/camera_info',
                             '/privacy_zones')
    rospy.spin()
