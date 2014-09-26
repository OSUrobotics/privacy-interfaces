#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PolygonStamped, PoseArray, PoseStamped, Vector3Stamped, PointStamped
from image_geometry import PinholeCameraModel
import cv2
import cv_bridge
import numpy
import tf
from tf.transformations import *


class Projector():
    def __init__(self):
        self.need_bounds = True
        self.need_info = True
        self.need_amcl = True

        self.bridge = cv_bridge.CvBridge()
        
        self.model = PinholeCameraModel()
        
        self.lis = tf.TransformListener()

        # Subscribers
        rospy.Subscriber('/object_bounds', PolygonStamped, self.bounds_callback)
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.info_callback)
        rospy.Subscriber('/particlecloud', PoseArray, self.amcl_callback)
        rospy.Subscriber('/camera/rgb/image_color', Image, self.image_callback, queue_size=1)
        
        self.pub_image = rospy.Publisher('/camera/rgb/image_color_filtered', Image)
        
    def bounds_callback(self, bounds):
        if self.need_bounds:
            self.bounds = bounds
            self.need_bounds = False

    def info_callback(self, info):
        if self.need_info:
            self.info = info
            self.model.fromCameraInfo(info)
            self.need_info = False

    def amcl_callback(self, amcl):
        """ Continuously update AMCL pose array. """
        self.amcl = amcl
        if self.need_amcl: 
            self.need_amcl = False

    def image_callback(self, image):
        """ Filter image! """
        if not any([self.need_bounds, self.need_info, self.need_amcl]):
            array = self.bridge.imgmsg_to_cv2(image, "bgr8")

            robot_frame = self.amcl.header.frame_id
            obj_frame = self.bounds.header.frame_id
            camera_frame = self.info.header.frame_id

            # Get object position
            translation, rotation = self.lis.lookupTransform(robot_frame, obj_frame, rospy.Time(0))
            obj = PointStamped()
            obj.header.frame_id = robot_frame
            [obj.point.x,
             obj.point.y,
             obj.point.z] = translation

            #### MARK OBJECT FROM ESTIMATED POSE ####
            polygon = []
            for point in self.bounds.polygon.points:

                point_stamped = PointStamped()
                point_stamped.header.frame_id = obj_frame
                point_stamped.header.stamp = rospy.Time(0)
                point_stamped.point.x = point.x
                point_stamped.point.y = point.y
                point_stamped.point.z = point.z

                self.lis.waitForTransform(camera_frame, obj_frame, rospy.Time(0), rospy.Duration(3.0))
                point_cam = self.lis.transformPoint(camera_frame, point_stamped)

                if point_cam.point.z < 0:
                    rospy.logerr('Beware: back projection!')
                    polygon = [[0,0],[0,0],[0,0],[0,0]]
                    break

                (u, v) = self.model.project3dToPixel((point_cam.point.x,
                                                      point_cam.point.y,
                                                      point_cam.point.z))

                u = max(min(u, array.shape[1]-1), 1)
                v = max(min(v, array.shape[0]-1), 1)
                polygon.append([int(u), int(v)])
                
            cv2.fillConvexPoly(array,
                               numpy.asarray(polygon),
                               (255, 0, 0))


            #### MARK OBJECT FROM ALL AMCL POSES ####

            # Get transform from base frame to camera frame
            trans_base, rot = self.lis.lookupTransform(robot_frame, '/base_link', rospy.Time(0))
            trans_cam, rot = self.lis.lookupTransform(robot_frame, camera_frame, rospy.Time(0))
            translation = tuple(c-b for b, c in zip(trans_base, trans_cam))
            #translation = (0, 0, 0.5)

            # For each pose...
            for pose in self.amcl.poses:

                # Transform pose up from the base to the camera
                camera = PoseStamped()
                camera.header.frame_id = camera_frame
                camera.header.stamp = rospy.Time(0)
                camera.pose.position.x = pose.position.x + translation[0]
                camera.pose.position.y = pose.position.y + translation[1]
                camera.pose.position.z = pose.position.z + translation[2]
                camera.pose.orientation = pose.orientation
                
                # Add offsets for bounds and project to camera point-of-view
                rays = []
                for point in self.bounds.polygon.points:
                    vertex = PointStamped()
                    vertex.header.frame_id = obj.header.frame_id
                    vertex.point.x = obj.point.x + point.x
                    vertex.point.y = obj.point.y + point.y
                    vertex.point.z = obj.point.z + point.z
                    ray = project_point_onto_pose(camera, vertex)
                    rays.append(ray)
                    
                # Add to list of polygons in UV
                polygon = []
                for ray in rays:
                    if ray.vector.z < 0:  # no back-projections!
                        polygon = [[0,0], [0,0], [0,0]]
                        break
                    (u, v) = self.model.project3dToPixel((ray.vector.x,
                                                          ray.vector.y,
                                                          ray.vector.z))
                    u = max(min(u, array.shape[1]-1), 1)
                    v = max(min(v, array.shape[0]-1), 1)
                    polygon.append([int(u), int(v)])

                cv2.fillConvexPoly(array,
                                   numpy.asarray(polygon),
                                   (0, 0, 255))
            
            image_new = self.bridge.cv2_to_imgmsg(array, "bgr8")
            image_new.header.stamp = rospy.Time.now()
            self.pub_image.publish(image_new)


def project_point_onto_pose(pose_stamped, point_stamped):

    # Find relative position of object in /map frame
    v_map = numpy.array([point_stamped.point.x - pose_stamped.pose.position.x,
                         point_stamped.point.y - pose_stamped.pose.position.y,
                         point_stamped.point.z - pose_stamped.pose.position.z])

    # Calculate pose_stamped heading vector
    heading = Vector3Stamped()
    heading.header.frame_id = '/map'
    q_pose = numpy.asarray([pose_stamped.pose.orientation.x,
                            pose_stamped.pose.orientation.y,
                            pose_stamped.pose.orientation.z,
                            pose_stamped.pose.orientation.w])
    R_pose = quaternion_matrix(q_pose)
    v_heading = numpy.matrix([1, 0, 0, 1]) * numpy.matrix(R_pose).I
    v_heading /= v_heading[0, 3]  # ensure homogeneity isn't messing stuff up
    v_heading = v_heading[0, 0:3].tolist()[0]  # convert from homogeneous to...not
    [heading.vector.x,
     heading.vector.y,
     heading.vector.z] = v_heading

    # Unit vectors in pose_stamped pose "frame"
    k_pose = numpy.array(v_heading)
    j_pose = numpy.array([0, 0, -1])
    i_pose = numpy.cross(j_pose, k_pose)
    
    # Convert to pose "frame" from /map frame
    obj_rel = Vector3Stamped()
    obj_rel.header.frame_id = 'POSE_STAMPED_POSE_FRAME'
    obj_rel.vector.x = numpy.dot(v_map, i_pose)
    obj_rel.vector.y = numpy.dot(v_map, j_pose)
    obj_rel.vector.z = numpy.dot(v_map, k_pose)
    return obj_rel


if __name__ == "__main__":

    rospy.init_node('project_onto_possible_cameras')
    projector = Projector()
    rospy.spin()
