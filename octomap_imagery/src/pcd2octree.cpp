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
  
}
