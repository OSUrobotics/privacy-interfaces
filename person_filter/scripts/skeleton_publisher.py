#!/usr/bin/env python

import rospy
import tf
from person_filter.msg import SkeletonJoint, Skeleton, SkeletonArray

def flatten(thing):
    """ Flatten nested dictionaries. """
    try:
        for item in thing.iteritems():
            yield item[0]  # the key
            for item_nested in flatten(item[1]):
                yield item_nested
    except AttributeError:  # from calling "iteritems()" on a non-dict!
        yield thing
       

# Joint tree in dict form
def recite_joints(suffix=''):
    return {'head'+suffix: 
            {'neck'+suffix: 
             {'left_shoulder'+suffix:
                  {'left_elbow'+suffix: 'left_hand'+suffix}, 
              'torso'+suffix:
                  {'left_hip'+suffix:
                       {'left_knee'+suffix: 'left_foot'+suffix}, 
                   'right_hip'+suffix:
                       {'right_knee'+suffix: 'right_foot'+suffix}}, 
              'right_shoulder'+suffix:
                  {'right_elbow'+suffix: 'right_hand'+suffix}}}}

def build_joint(joint_name, child_joints, parent_frame, listener):
    """ Build a single joint in XYZ space using a tf listener. """
    # Get the transform
    listener.waitForTransform(parent_frame, joint_name, rospy.Time(0), rospy.Duration(1.0))
    transform = listener.lookupTransform(parent_frame, joint_name, rospy.Time(0))

    # Build the msg
    joint = SkeletonJoint()
    joint.joint_name = joint_name
    joint.child_joints = child_joints
    joint.xyz = list(transform[0])  # xyz coordinates
    return joint
    
    
def build_skeleton(joints, parent_frame, listener, skeleton):
    """ Recursively (woohoo!) build skeleton of joint positions in XYZ space. """
    try:  # for a dict
        for joint in joints.iteritems():
            joint_name = joint[0]  # the joint name
            child_joints = joint[1]  # downstream joints

            # Create this joint
            skeleton.joints.append(
                build_joint(joint_name, child_joints, parent_frame, listener))

            # Create all downstream joints (recursively)
            build_skeleton(joint_children, parent_frame, listener, skeleton)

    except AttributeError:  # for a list or string
        if isinstance(joints, basestring):  # for a string
            skeleton.joints.append(
                build_joint(joints, '', parent_frame, listener))  # leaf of joint tree

        else:  # for list
            for joint_or_joints in joints:
                build_skeleton(joint_or_joints, parent_frame, listener, skeleton)
    

if __name__ == "__main__":
    rospy.init_node('skeleton_publisher')

    # TF Listener
    listener = tf.TransformListener()
    rospy.sleep(1.0)  # give listener time to initialize

    # Publisher for joint positions
    pub = rospy.Publisher('/skeletons', SkeletonArray)
    
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        skeleton_array = SkeletonArray()  # clear skeleton array
        frames = listener.getFrameStrings()
        joints_all = []
        for n in range(1,10):
            suffix = '_' + str(n)
            joints = list(flatten(recite_joints(suffix)))  # generate joint names
            if all([joint in frames for joint in joints]):
                joints_all.append(joints)
            else:
                print 'This many skeletons: ' + str(n-1)
                break

        transforms_all = []
        for joints in joints_all:
            skeleton = Skeleton()  # clear skeleton
            build_skeleton(joints, '/camera_rgb_optical_frame', listener, skeleton)
            skeleton_array.skeletons.append(skeleton)

        skeleton_array.header.frame_id = '/camera_rgb_optical_frame'
        skeleton_array.header.stamp = rospy.Time.now()
        if len(skeleton_array.skeletons) > 0:  # if there are any skeletons
            pub.publish(skeleton_array)
        r.sleep()
