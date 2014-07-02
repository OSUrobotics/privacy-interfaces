// Convert PointCloud2 messages to Octomaps and publish out.

#include <iostream>
#include <stdlib.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>  // allows subscribing/publishing PCL types as ROS msgs

// PCL
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

// Iterator stuff
#include <iterator>


// Globals :-(
float leaf_size = 0.05;  // 2cm default minimum leaf size
ros::Publisher pub;


void cloud_callback (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  // Initial cloud size
  std::cerr << "PointCloud before insertion: " << cloud_in->width * cloud_in->height 
	    << " data points (" << pcl::getFieldsList (*cloud_in) << ")." << std::endl;

  // Load into *color* octree
  octomap::Pointcloud octomapCloud;
  pointcloudPCLToOctomap(*cloud_in, octomapCloud);
  octomap::point3d sensor_origin (0, 0, 0);
  octomap::ColorOcTree color_octree (leaf_size);
  color_octree.insertPointCloud(octomapCloud, sensor_origin);

  // Add in the colors
  pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it;
  for (it = cloud_in->begin(); it != cloud_in->end(); ++it) 
    {
      if (!isnan (it->x) && !isnan (it->y) && !isnan (it->z))  // Check if the point is invalid
	  color_octree.averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
    }
  color_octree.updateInnerOccupancy();  // updates inner node colors, too

  color_octree.prune();
  color_octree.toMaxLikelihood();

  octomap_msgs::Octomap octomap;
  octomap.header.stamp = ros::Time::now();
  octomap.header.frame_id = "/camera_rgb_optical_frame";
  octomap_msgs::fullMapToMsg (color_octree, octomap);

  pub.publish (octomap);
  
}


int main (int argc, char** argv)
{
  if (argc >= 2)
    leaf_size = atof(argv[1]);  // set Octree minimum leaf size

  ros::init(argc, argv, "clouds2Octomaps");
  ros::NodeHandle node;

  pub = node.advertise <octomap_msgs::Octomap> ("octomap_out", 5);
  ros::Subscriber sub = node.subscribe("cloud_in", 5, cloud_callback);

  ros::spin();
}
