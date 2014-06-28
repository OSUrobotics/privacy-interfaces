// 
#include <iostream>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>

// TF
//#include <tf/transform_listener.h>

// PCL
//#include <pcl_ros/transforms.h>
//#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
//#include <pcl_conversions/pcl_conversions.h>

// Iterator stuff
#include <iterator>

// Octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
//#include <octomap_msgs/Octomap.h>

using namespace std;
using namespace pcl;
using namespace ros;


int main (int argc, char** argv)
{
  // Initialize node
  ros::init(argc, argv, "octomap2Image");
  ros::NodeHandle nh;

  // Create camera model (based on Kinect)
  /*
  sensor_msgs::CameraInfo camInfo;
  camInfo.header.frame_id = "/camera_rgb_optical_frame";
  camInfo.height = 480;
  camInfo.width = 640;
  camInfo.distortion_model = "plumb_bob";

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camInfo);
  cv::Point3d ray;
  cv::Point2d uv_rect (0, 0);
  ray = model.projectPixelTo3dRay(uv_rect);
  cout << ray.x << ray.y << ray.z << endl;
  */

  // Import cloud from .PCD file
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  reader.read ("/home/ruebenm/workspaces/privacy_ws/src/octomap_imagery/data/pcd/wheelchair_0.pcd", *cloud_in);
  std::cout << "PointCloud is " << cloud_in->width << " wide x " << cloud_in->height << " high." << std::endl;

  // Load into *color* octree
  octomap::Pointcloud octomapCloud;
  pointcloudPCLToOctomap(*cloud_in, octomapCloud);
  octomap::point3d sensor_origin (0, 0, 0);
  octomap::ColorOcTree color_octree (0.05f);
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
  string fn_color = "/home/ruebenm/workspaces/privacy_ws/src/octomap_imagery/data/octree/tree_color.ot";
  string fn_color_bin = "/home/ruebenm/workspaces/privacy_ws/src/octomap_imagery/data/octree/tree_color.bt";
  ofstream file_color(fn_color.c_str(), ios_base::binary);
  ofstream file_color_bin(fn_color_bin.c_str(), ios_base::binary);
  color_octree.write (file_color);
  color_octree.writeBinary (file_color_bin);

  // Visualize *in color*
  // (just type "octovis <filename>")

  // Project onto image plane
  sensor_msgs::Image image;
  image.height = 480;
  image.width = 640;
  image.step = image.width * 3;  // for the 3 colors
  image.encoding = "bgr8";
  image.data.resize(image.height * image.step);  // get the size right, y'all
  std::vector<uint8_t>::iterator it_im;

  bool success;
  octomap::point3d origin (0,0,0), direction (0,0,1), end;
  octomap::ColorOcTreeNode* node;
  std::cout << "Start filling Image msg..." << std::endl;
  unsigned short int foo = 0;
  for (it_im = image.data.begin(); it_im != image.data.end(); ++it_im, foo++)
    {
      success = color_octree.castRay(origin, direction, end);
      node = color_octree.search(end);
      //std::cout << success << node->getColor() << std::endl;
      *it_im = foo;
    }
  std::cout << "Done." << std::endl;


  // Display image...somehow
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("image", 5);  // topic name works with image_view without remapping!
  ros::Rate loop_rate(10);
  while (ros::ok())
    {
      pub.publish(image);
      ros::spinOnce();
      loop_rate.sleep();
    }
  
}
