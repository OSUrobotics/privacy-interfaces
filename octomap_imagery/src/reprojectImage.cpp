// Gets some Kinect data from bag file, then reprojects it (to prove a point).

#include <iostream>
#include <stdlib.h>

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

// STL
#include <iterator>


int main (int argc, char** argv)
{
  // Initialize node
  ros::init(argc, argv, "reprojectImage");
  int bin_factor = atoi(argv[1]);
  ros::NodeHandle nh;

  // Initialize rosbag stuff
  rosbag::Bag bag("/home/ruebenm/workspaces/privacy_ws/src/plane_shaver/bags/my-desk.bag");
  rosbag::View view_image(bag, rosbag::TopicQuery("/camera/rgb/image_color"));
  rosbag::View view_info(bag, rosbag::TopicQuery("/camera/rgb/camera_info"));

  // De-bag an Image
  sensor_msgs::Image::ConstPtr image_in = view_image.begin() -> instantiate<sensor_msgs::Image> ();
  if (image_in != NULL)
    std::cout << "Image de-bagged with frame_id: " << image_in->header.frame_id << std::endl;
  
  // De-bag the CameraInfo
  sensor_msgs::CameraInfo::ConstPtr info = view_info.begin() -> instantiate<sensor_msgs::CameraInfo> ();
  if (info != NULL)
    std::cout << "Info de-bagged with distortion model: " << info->distortion_model << std::endl;

  // Add binning
  sensor_msgs::CameraInfo::Ptr info_binned (new sensor_msgs::CameraInfo ());
  info_binned->header = info->header;
  info_binned->height = info->height;
  info_binned->width = info->width;
  info_binned->distortion_model = info->distortion_model;
  info_binned->D = info->D;
  info_binned->K = info->K;
  info_binned->R = info->R;
  info_binned->P = info->P;
  info_binned->binning_x = bin_factor;
  info_binned->binning_y = bin_factor;
  info_binned->roi = info->roi;

  // Set up downsampling via "binning" options
  image_geometry::PinholeCameraModel model, model_binned;
  model.fromCameraInfo(info);
  model_binned.fromCameraInfo(info_binned);
  cv::Point3d ray;
  cv::Point2d uv_dst, uv_src;
  
  // Project onto image plane
  sensor_msgs::Image image;
  image.height = 480 / info_binned->binning_y;
  image.width = 640 / info_binned->binning_x;
  image.step = image.width * 3;  // for the 3 colors
  image.encoding = "bgr8";
  image.data.resize(image.height * image.step);  // get the size right, y'all
  std::cout << "Created new Image msg with resolution " << image.width << " x " << image.height << std::endl;
  //std::vector<uint8_t>::iterator it_im;

  std::cout << "Start filling Image msg..." << std::endl;
  int rgb = 0;
  for (uv_dst.x = 0; uv_dst.x != image.width; uv_dst.x++)
    {
      for (uv_dst.y = 0; uv_dst.y != image.height; uv_dst.y++)
	{
	  std::cout << "(" << uv_dst.x << ", " << uv_dst.y << ")" << std::endl;
	  ray = model_binned.projectPixelTo3dRay(uv_dst);
	  uv_src = model.project3dToPixel(ray);
	  for (rgb = 0; rgb != 3; rgb++)
	    image.data[(uv_dst.y * image.step) + (uv_dst.x * 3) + rgb] = image_in->data[(uv_src.y * image_in->step) + (uv_src.x * 3) + rgb];
	}
    }
  std::cout << "Done." << std::endl;

  // Display image...somehow
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image", 5);  // topic name works with image_view without remapping!
  ros::Publisher pub_binned = nh.advertise<sensor_msgs::Image>("image_binned", 5);
  ros::Rate loop_rate(10);
  while (ros::ok())
    {
      pub.publish(*image_in);
      pub_binned.publish(image);
      ros::spinOnce();
      loop_rate.sleep();
    }


}
