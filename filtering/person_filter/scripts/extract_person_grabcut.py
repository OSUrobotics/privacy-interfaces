#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from person_filter.msg import SkeletonArray
import numpy
import cv
import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters


class SkeletonSketcher():
    def __init__(self, image_topic, depth_topic, skeletons_topic):

        self.bridge = CvBridge()

        # Synchronized subscribers
        sub_rgb = message_filters.Subscriber(image_topic, Image)
        sub_depth = message_filters.Subscriber(depth_topic, Image)
        sub_skeletons = message_filters.Subscriber(skeletons_topic, SkeletonArray)
        ts = message_filters.TimeSynchronizer([sub_rgb, sub_depth, sub_skeletons], 1)
        ts.registerCallback(self.callback)


    def callback(self, image, depth, skeleton_array):
        # Prep the depth image
        depth_mid = self.bridge.imgmsg_to_cv(depth)
        depth_cv = numpy.asarray(depth_mid)

        #for erosion, the bigger the shape, the more it will delete
        kernel = numpy.ones((15,15),numpy.uint8)

        #for graphcut, dont change anything
        bgdModel = numpy.zeros((1,65),numpy.float64)
        fgdModel = numpy.zeros((1,65),numpy.float64)
        
        # Prep the RGB image
        try:
            image_cv = self.bridge.imgmsg_to_cv2(image)
        except CvBridgeError, e:
            print e
                
        # Extract all the people
        for s in range(len(skeleton_array.skeletons)):
            mask = self.get_skeleton_mask(skeleton_array.skeletons[s],depth_cv)
				
            #####TESTING####################################################################
            #mask2 = numpy.copy(mask)
	    #image_raw = numpy.copy(image_cv)        
            # show the definite frontground
            #mask2 = numpy.where((mask==2)|(mask==0),0,1).astype('uint8')
            #image_raw = image_cv*mask2[:,:,numpy.newaxis]
            #cv2.imwrite("/home/dain/Desktop/Test/MaskBeforeErosion.jpg",image_raw)
            ##################################################################

            #get the new eroded mask
            mask = self.get_new_eroded_mask(kernel,mask)
				
            ####TESTING################################################
            # show the definite frontground
            #mask2 = numpy.where((mask==2)|(mask==0),0,1).astype('uint8')
            #image_raw = image_cv*mask2[:,:,numpy.newaxis]
            #cv2.imwrite("/home/ruebenm/Desktop/MaskAfterErosion.jpg",image_raw)
            #rospy.signal_shutdown(True)  # if you only want one image 
            ###############################################################333

            cv2.grabCut(image_cv,mask,None,bgdModel,fgdModel,1,cv2.GC_INIT_WITH_MASK)
            
            mask = numpy.where((mask==2)|(mask==0),0,1).astype('uint8')
            image_cv = image_cv*mask[:,:,numpy.newaxis]
            
            # in case we need to publish it
            try:
                image = self.bridge.cv2_to_imgmsg(image_cv)
            except CvBridgeError, e:
                print e
                
            #####testing
            #cv2.imwrite("/home/dain/Desktop/Test/result.jpg",image_cv)
            cv.ShowImage('Image with Skeletons', cv.fromarray(image_cv))
            cv2.waitKey(5)
            #####	


    def get_skeleton_mask(self,skeleton,depth_cv):
        """
        Get the mask for graphcut using skeleton and depth image. The
        mask has 3 layer: the first layer is definite background,
        which cover the whole image, the next layer is possible
        background, which is a rectangle around the person (the
        rectangle is fromed by finding the min and max of u and v),
        the last layer is definite frontground, which is the merge of
        the depth image of a person and the joints from the skeleton
        
        Input: skeleton, cv_depth_image Output: mask_array
        """
        # for finding the max and min in u,v, and z
        coor = numpy.zeros(4)
        ran = numpy.zeros(2)	
        
        mask = numpy.zeros(depth_cv.shape,numpy.uint8)
        
        # helper array
        # need 2 helpers as I don't do the possible frontground anymore
        helper = numpy.zeros (depth_cv.shape,numpy.uint8)
        helper1 = numpy.zeros (depth_cv.shape,numpy.uint8)

        for j in range(len(skeleton.joints)):
            u = int(skeleton.joints[j].uv[0])
            v = int(skeleton.joints[j].uv[1])
            z = skeleton.joints[j].xyz[2]
            
            #flag to indicate if it is the first joint or not
            if j == 0:
                flag = True
            else:
                flag = False
                
            coor = self.get_corner(coor,u,v,flag)
            ran = self.get_depth_range (ran,z,flag)

            #get the joint and mark it
            if skeleton.joints[j].joint_name == "head_1":
                helper1[v-int(130/z) : v+int(100/z), u-int(50/z) : u+int(50/z)] = 3
                
            elif skeleton.joints[j].joint_name == "torso_1":
                helper1[v-int(120/z) : v+int(120/z), u-int(120/z) : u+int(120/z)] = 3
				
            elif (skeleton.joints[j].joint_name == "right_hand_1" or
                  skeleton.joints[j].joint_name == "left_hand_1" or
                  skeleton.joints[j].joint_name == "left_foot_1" or
                  skeleton.joints[j].joint_name == "right_foot_1" or
                  skeleton.joints[j].joint_name == "left_elbow_1" or
                  skeleton.joints[j].joint_name == "right_elbow_1" ):
                helper1[v-int(100/z) : v+int(100/z), u-int(100/z) : u+int(100/z)] = 3
										
            else :
                helper1[v-int(100/z) : v+int(100/z), u-int(50/z) : u+int(50/z)] = 3
		
        #get the rectangle around the body, which is the possible background
        mask[coor[0]-int(140/z):coor[1]+int(110/z),coor[2]-int(90/z):coor[3]+int(90/z) ]=2

        # mark the possition of depth into the helper array in order to merge them later 
        # Note: depth values are in millimeters!
        # add and minute 0.50 because the depth and rgb out of sync, can delete it later
        helper = numpy.where ((depth_cv > ran[0] * 1000 - 0.50) & (depth_cv < ran[1] * 1000 + 0.50),3,helper)

        # merge the joint and depth together to get definite frontground
        mask = numpy.where ((helper1==3)&(helper==3),1,mask)

        return mask

    def get_new_eroded_mask(self,kernel,mask):
        """ 
        This is for erosion. I want to convert the rgb
        image to black, dark sliver, bright sliver,
        and white based on the mask as it will be more
        accurate.

        Input: kernel, mask_array Output: mask_array
        """
        image_helper = numpy.zeros(numpy.append(mask.shape,3),numpy.uint8)
		
        # from rgb to the the new color scheme
        image_helper[mask == 0 ] = (0,0,0)
        image_helper[mask == 1 ] = (255,255,255)
        image_helper[mask == 2 ] = (100,100,100)
        image_helper[mask == 3 ] = (200,200,200)
        
        ####### FOR TESTING ######
        #cv.ShowImage('mask before erosion', cv.fromarray(image_helper))
        #cv2.waitKey(5)
        ##########################

        image_helper = cv2.erode(image_helper,kernel,iterations = 1)

        ####### FOR TESTING ######
        #cv.ShowImage('mask after erosion', cv.fromarray(image_helper))
        #cv2.waitKey(5)
        ##########################

        # get back the new mask after erosion
        mask[numpy.where(numpy.all(image_helper == [0,0,0], axis=-1))] = 0
        mask[numpy.where(numpy.all(image_helper == [255,255,255], axis=-1))] = 1
        mask[numpy.where(numpy.all(image_helper == [100,100,100], axis=-1))] = 2
        mask[numpy.where(numpy.all(image_helper == [200,200,200], axis=-1))] = 3
        
        return mask

    def get_corner (self, coor, u, v, flag):
        """ 
        Get the min and max values of u and v
        coor[0] = top, coor[1] = bot, coor[2] = left, coor[3] = right
        
        """
        if flag:
            coor[0] = v
            coor[1] = v
            coor[2] = u
            coor[3] = u
        else:
            if v < coor[0]:
                coor[0] = v
            if v > coor[1]:
                coor[1] = v
            if u < coor[2]:
                coor[2] = u
            if u > coor[3]:
                coor[3] = u
        return coor

    def get_depth_range (self, ran, z,flag):
        #NOTE: can combine with get_corner to make it a little shorter and maybe faster
        """ 
        Get the min and max value of z
        ran[0] = front, ran[1] = back
        
        """
        if flag:
            ran[0] = z
            ran[1] = z
        else:
            if z < ran[0]:
                ran[0] = z
            if z > ran[1]:
                ran[1] = z
        return ran


if __name__ == "__main__":
    rospy.init_node('skeletons_onto_image')
    skeleton_sketcher = SkeletonSketcher('/camera/rgb/image_color/sync', 
                                         '/camera/depth_registered/image_raw/sync',
                                         '/skeletons_uv/sync')

    rospy.spin()
    
