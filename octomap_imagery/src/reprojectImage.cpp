// Gets some Kinect data from bag file, then reprojects it (to prove a point).

#include <iostream>

// ROS
#include <ros/ros.h>

// Image 
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>

// rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>

// boost
#include <boost/foreach.hpp>


int main (int argc, char** argv)
{
  // Initialize node
  ros::init(argc, argv, "reprojectImage");
  ros::NodeHandle nh;

  // Initialize rosbag stuff
  rosbag::Bag bag("/home/ruebenm/workspaces/privacy_ws/src/plane_shaver/bags/my-desk.bag");
  rosbag::View view_image(bag, rosbag::TopicQuery("/camera/rgb/image_color"));
  rosbag::View view_info(bag, rosbag::TopicQuery("/camera/rgb/camera_info"));

  // De-bag an Image
  sensor_msgs::Image::ConstPtr image = view_image.begin() -> instantiate<sensor_msgs::Image> ();
  if (image != NULL)
    std::cout << "Image de-bagged with frame_id: " << image->header.frame_id << std::endl;
  
  // De-bag the CameraInfo
  sensor_msgs::CameraInfo::ConstPtr info = view_info.begin() -> instantiate<sensor_msgs::CameraInfo> ();
  if (info != NULL)
    std::cout << "Info de-bagged with distortion model: " << info->distortion_model << std::endl;

}
