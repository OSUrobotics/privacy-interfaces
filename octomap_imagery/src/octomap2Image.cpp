// Constructs an octree from a single PointCloud2 msg, then reprojects to a camera view.
// INPUTS:
// 1) Resolution of octree in meters (try 2-5cm)
// 2) Downsampling factor of image, which is originally 640x480 (e.g. choose "10" for a 64x48 image)
// 3) If True, transform cloud to '/map' frame before inserting into octree. Allows you to pretend the camera is moving using the static_transform_publisher node.


#include <iostream>
#include <stdlib.h>
#include <string>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

// TF
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

// Iterator stuff
#include <iterator>

// Octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
//#include <octomap_msgs/Octomap.h>

// rosbag
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;
using namespace pcl;
using namespace ros;


int main (int argc, char** argv)
{
  // Initialize node
  ros::init(argc, argv, "octomap2Image");
  ros::NodeHandle nh;

  // Argument checking
  if (argc != 4) {
    std::cerr << "Supply three arguments! See comments for what they mean." << std::endl;
    return -1;
  }

  // Unpack inputs
  double res = atof(argv[1]);
  int bin_factor = atoi(argv[2]);
  bool has_tf = atoi(argv[3]);

  // Initialize rosbag stuff
  rosbag::Bag bag("/home/ruebenm/workspaces/privacy_ws/src/plane_shaver/bags/my-desk.bag");
  rosbag::View view_image(bag, rosbag::TopicQuery("/camera/rgb/image_color"));
  rosbag::View view_info(bag, rosbag::TopicQuery("/camera/rgb/camera_info"));
  rosbag::View view_cloud(bag, rosbag::TopicQuery("/camera/depth_registered/points"));

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
  image_geometry::PinholeCameraModel model_binned;
  model_binned.fromCameraInfo(info_binned);
  
  // De-bag a PointCloud2
  sensor_msgs::PointCloud2::Ptr cloud = view_cloud.begin() -> instantiate<sensor_msgs::PointCloud2> ();
  if (cloud != NULL)
    std::cout << "Point cloud de-bagged with frame_id: " << cloud->header.frame_id << std::endl;

  // Transform?
  sensor_msgs::PointCloud2 cloud_tf;
  if (has_tf) 
    {
      tf::TransformListener listener;
      bool can_transform;
      string frame_new = "/map";
      can_transform = listener.waitForTransform(cloud->header.frame_id, 
						frame_new,
						ros::Time(0), 
						ros::Duration(3.0));
      if (can_transform)
	{
	  cloud->header.stamp = ros::Time(0);
	  pcl_ros::transformPointCloud(frame_new, 
				       *cloud, 
				       cloud_tf, 
				       listener);
	  std::cout << "Cloud transformed to frame: " << frame_new << std::endl;
	}
      else 
	{
	  std::cerr << "Could not transform! :-(" << std::endl;
	  return -1;
	}
      
    }


  // Convert PointCloud2 to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (has_tf)
    fromROSMsg(cloud_tf, *cloud_in);
  else
    fromROSMsg(*cloud, *cloud_in);
    
  std::cout << "PointCloud is " << cloud_in->width << " wide x " << cloud_in->height << " high." << std::endl;

  // Load into *color* octree
  octomap::Pointcloud octomapCloud;
  pointcloudPCLToOctomap(*cloud_in, octomapCloud);
  octomap::point3d sensor_origin (0, 0, 0);
  octomap::ColorOcTree color_octree (res);
  color_octree.insertPointCloud(octomapCloud, sensor_origin);

  // Add in the colors
  pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
  for (it = cloud_in->begin(); it != cloud_in->end(); ++it) 
    {
      if (!isnan (it->x) && !isnan (it->y) && !isnan (it->z))  // Check if the point is invalid
	  color_octree.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
    }
  color_octree.updateInnerOccupancy();  // updates inner node colors, too
  
  // Save *color* octree
  /*
  string fn_color = "/home/ruebenm/workspaces/privacy_ws/src/octomap_imagery/data/octree/tree_color.ot";
  string fn_color_bin = "/home/ruebenm/workspaces/privacy_ws/src/octomap_imagery/data/octree/tree_color.bt";
  ofstream file_color(fn_color.c_str(), ios_base::binary);
  ofstream file_color_bin(fn_color_bin.c_str(), ios_base::binary);
  color_octree.write (file_color);
  color_octree.writeBinary (file_color_bin);
  */

  // Visualize *in color*
  // (just type "octovis <filename>")

  // Initialize Image msg
  sensor_msgs::Image image;
  image.height = 480 / info_binned->binning_y;
  image.width = 640 / info_binned->binning_x;
  image.step = image.width * 3;  // for the 3 colors
  image.encoding = "bgr8";
  image.data.resize(image.height * image.step);  // get the size right, y'all
  std::cout << "Created new Image msg with resolution " << image.width << " x " << image.height << std::endl;
  //std::vector<uint8_t>::iterator it_im;

  // Fill Image msg (project octree onto camera plane)
  cv::Point3d ray;
  cv::Point2d uv_dst;
  octomap::point3d end;  // NEED TO UPDATE "origin" WITH TF FRAME
  tf::Vector3 origin (0,0,0);
  bool success;
  octomap::ColorOcTreeNode* node;
  unsigned short int color[3];
  int bgr;

  // Publish image
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image", 5);  // topic name works with image_view without remapping!
  ros::Rate loop_rate(100);
  std::cout << "Filling Image msg..." << std::endl;

  // Transformer
  tf::TransformListener listener;
  bool can_transform;
  string frame_old = "/map";
  string frame_new = "/camera_rgb_optical_frame";
  can_transform = listener.waitForTransform(frame_old, 
					    frame_new,
					    ros::Time(0), 
					    ros::Duration(3.0));
  if (!can_transform) 
    {
      std::cerr << "Could not transform! :-(" << std::endl;
      return -1;
    }
  tf::StampedTransform tf;
      
  

  while (ros::ok())
    {

      // Look up transforms if necessary
      if (has_tf)
	{
	  listener.lookupTransform(frame_new, 
				   frame_old,
				   ros::Time(0), 
				   tf);
	  std::cout << "Looked up transform from frame: " << frame_old << " to frame: " << frame_new << std::endl;
	}
      else
	{
	  tf.setIdentity();  // transform does nothing
	}

      for (uv_dst.x = 0; uv_dst.x != image.width; uv_dst.x++)
	{
	  for (uv_dst.y = 0; uv_dst.y != image.height; uv_dst.y++)
	    {
	      //std::cout << "UV: (" << uv_dst.x << ", " << uv_dst.y << ")" << std::endl;
	      ray = model_binned.projectPixelTo3dRay(uv_dst);  // outputs cv::Point3d
	      //std::cout << "Ray: (" << ray.x << ", " << ray.y << ", " << ray.z << ")" << endl;
	      tf::Vector3 direction (ray.x, ray.y, ray.z);
	      //std::cout << "Direction: (" << direction.x() << ", " << direction.y() << ", " << direction.z() << ")" << endl;

	      // Do transforms
	      tf::Vector3 origin_tf = tf * origin;
	      tf::Vector3 direction_tf = tf * direction;
	      octomap::point3d origin_final (origin_tf.x(), origin_tf.y(), origin_tf.z());
	      octomap::point3d direction_final (direction_tf.x(), direction_tf.y(), direction_tf.z());
	      

	      success = color_octree.castRay(origin_final, direction_final, end);
	      
	      if (success)
		{
		  node = color_octree.search(end);
		  //std::cout << success << "  " << node->getColor() << std::endl;
		  color[0] = node->getColor().b;  // blue
		  color[1] = node->getColor().g;  // green
		  color[2] = node->getColor().r;  // red
		}
	      else
		color[0] = color[1] = color[2] = 0;  // black
	      
	      for (bgr = 0; bgr != 3; bgr++)
		image.data[(uv_dst.y * image.step) + (uv_dst.x * 3) + bgr] = color[bgr];
	      
	    }
	}
      std::cout << "Done. Publishing!" << std::endl;
      pub.publish(image);
      ros::spinOnce();
      loop_rate.sleep();
    }

  
}
