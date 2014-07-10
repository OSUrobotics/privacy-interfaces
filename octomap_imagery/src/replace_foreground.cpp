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

// for PCD file handling in particular
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Other stuff
#include <iostream>
#include <vector>
#include <stdlib.h>



// Globals :-(
float resolution = 0.02f;
pcl::visualization::ImageViewer viewer ("View Point Clouds as Images!");
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore;
bool is_first_cloud = true;


void replace_foreground (float resolution,
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

  std::vector<int> newPointIdxVector;

  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // Replace foreground points with background points
  std::cout << "Replacing this many points:" << newPointIdxVector.size() << std::endl;
  for (size_t i = 0; i < newPointIdxVector.size (); ++i)
    {
      cloud_fore->points[newPointIdxVector[i]].r = cloud_back->points[newPointIdxVector[i]].r;
      cloud_fore->points[newPointIdxVector[i]].g = cloud_back->points[newPointIdxVector[i]].g;
      cloud_fore->points[newPointIdxVector[i]].b = cloud_back->points[newPointIdxVector[i]].b;
      cloud_fore->points[newPointIdxVector[i]].x = cloud_back->points[newPointIdxVector[i]].x;
      cloud_fore->points[newPointIdxVector[i]].y = cloud_back->points[newPointIdxVector[i]].y;
      cloud_fore->points[newPointIdxVector[i]].z = cloud_back->points[newPointIdxVector[i]].z;
    }

  std::cout << "Replaced those points!" << std::endl;
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

  replace_foreground(resolution, cloud_back, cloud_fore);

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
