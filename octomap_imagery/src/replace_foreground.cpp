// Replaces foreground stuff with background stuff, essentially preventing an image from changing.
// Adapted from this tutorial: http://pointclouds.org/documentation/tutorials/octree_change.php
// ...as well as this one: http://pointclouds.org/documentation/tutorials/openni_grabber.php
// Matthew Rueben, Oregon State University

// ROS stuff
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/point_cloud.h>  // allows subscribing/publishing PCL types as ROS msgs

// PCL stuff
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/conversions.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/filters/impl/filter_indices.hpp>

// for PCD file handling in particular
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Other stuff
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <math.h>


// Globals :-(
float resolution = 0.02f;
pcl::visualization::ImageViewer viewer ("View Point Clouds as Images!");
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore;
bool is_first_cloud = true;


void replace_indices (std::vector<int> indices,
		      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back, 
		      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore)
{
  // Replace indexed foreground points with background points
  std::cout << "Replacing this many points:" << indices.size() << std::endl;
  for (size_t i = 0; i < indices.size (); ++i)
    {
      cloud_fore->points[indices[i]].r = cloud_back->points[indices[i]].r;
      cloud_fore->points[indices[i]].g = cloud_back->points[indices[i]].g;
      cloud_fore->points[indices[i]].b = cloud_back->points[indices[i]].b;
      cloud_fore->points[indices[i]].x = cloud_back->points[indices[i]].x;
      cloud_fore->points[indices[i]].y = cloud_back->points[indices[i]].y;
      cloud_fore->points[indices[i]].z = cloud_back->points[indices[i]].z;
    }

  std::cout << "Replaced those points!" << std::endl;
}


void redact_indices(std::vector<int> indices,
		      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore)
{
  // Replace indexed points with BLACK
  std::cout << "Redacting this many points:" << indices.size() << std::endl;
  for (size_t i = 0; i < indices.size (); ++i)
    {
      cloud_fore->points[indices[i]].r = 0;
      cloud_fore->points[indices[i]].g = 0;
      cloud_fore->points[indices[i]].b = 0;
    }

  std::cout << "Redacted those points!" << std::endl;
}


std::vector<int> filter_foreground (float resolution,
			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back, 
			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore)
{
  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree (resolution);

  // Add points from cloud_back to octree
  octree.setInputCloud (cloud_back);
  octree.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree.switchBuffers ();

  // Add points from cloud_fore to octree
  octree.setInputCloud (cloud_fore);
  octree.addPointsFromInputCloud ();

  std::vector<int> indices_fore;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (indices_fore);

  return indices_fore;
}


std::vector<int> filter_nans (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  std::vector<int> indices_nan;
  //indices_nan.resize(cloud->size());  // pre-allocate space
  int count = 0;
  for (int i = 0; i < cloud->points.size(); ++i)
    if (isnan(cloud->points[i].z))  // test if z-value is NaN
      {
	indices_nan.push_back(i);
	count++;
      }
  //indices_nan.resize(count);  // shrink back down

  return indices_nan;
}


void cloud_callback (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore)
{
  if (is_first_cloud)
    {
      std::cout << "Storing first cloud...";
      cloud_back = cloud_fore->makeShared();
      is_first_cloud = false;
      std::cout << "done!" << std::endl;
    }
  
  std::vector<int> indices_fore = filter_foreground(resolution, cloud_back, cloud_fore);
  std::vector<int> indices_nan = filter_nans(cloud_back);  // of BACKGROUND image!
  replace_indices(indices_fore, cloud_back, cloud_fore);
  replace_indices(indices_nan, cloud_back, cloud_fore);

  if (!viewer.wasStopped())
    viewer.showRGBImage(*cloud_fore);
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "replace_foreground");
  ros::NodeHandle node;

  // Octree resolution - side length of octree voxels
  if (argc == 2)
    resolution = atof(argv[1]);
  else
    {
      std::cerr << "Provide the octree resolution as an argument!" << std::endl;
      return -1;
    }
  
  ros::Subscriber sub = node.subscribe("cloud_in", 5, cloud_callback);

  ros::spin();
  
}
