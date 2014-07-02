// 
#include <iostream>
#include <stdlib.h>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>

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
  if (argc < 3)
    {
      std::cerr << "Input the ColorOcTree resolution, followed by at least one .PCD filename!" << std::endl;
      return -1;
    }

  float res = atof (argv[1]);

  // Import cloud(s) from .PCD file(s)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  octomap::point3d sensor_origin (0, 0, 0);
  octomap::ColorOcTree color_octree (res);
  for (int i = 2; i < argc; i++)
    {
      // Read in a cloud
      reader.read (argv[i], *cloud_in);
      std::cout << "PointCloud is " << cloud_in->width << " wide x " << cloud_in->height << " high." << std::endl;

      // Load into *color* octree
      octomap::Pointcloud octomapCloud;
      octomap::pointcloudPCLToOctomap(*cloud_in, octomapCloud);
      color_octree.insertPointCloud(octomapCloud, sensor_origin);
      
      // Add in the colors
      pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
      for (it = cloud_in->begin(); it != cloud_in->end(); ++it) 
	{
	  if (!isnan (it->x) && !isnan (it->y) && !isnan (it->z))  // Check if the point is invalid
	    color_octree.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
	}
      color_octree.updateInnerOccupancy();  // updates inner node colors, too

      cloud_in->clear();
      
    }

  
  // Save *color* octree
  string fn_color = "tree_color.ot";
  ofstream file_color(fn_color.c_str(), ios_base::binary);
  color_octree.write (file_color);

  // Visualize *in color*
  // (just type "octovis <filename>")
  
}
