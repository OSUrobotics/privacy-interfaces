// Replaces foreground stuff with background stuff, essentially preventing an image from changing.
// Adapted from this tutorial: http://pointclouds.org/documentation/tutorials/octree_change.php
// Matthew Rueben, Oregon State University

// ROS stuff
#include <ros/ros.h>
#include <ros/package.h>

// PCL stuff
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

// for PCD file handling in particular
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Other stuff
#include <iostream>
#include <vector>
#include <stdlib.h>

int main (int argc, char** argv)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZRGB> );
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore (new pcl::PointCloud<pcl::PointXYZRGB> );

  std::string path = ros::package::getPath("octomap_imagery") + "/data/pcd/replace_foreground/";

  if ((pcl::io::loadPCDFile<pcl::PointXYZRGB> (path + "test_background.pcd", *cloud_back) == -1) ||
      (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path + "test_foreground.pcd", *cloud_fore) == -1)) //* load the files
    {
      PCL_ERROR ("Couldn't read file -- are you in the right directory? \n");
      return (-1);
    }
  
  // Octree resolution - side length of octree voxels
  float resolution;
  if (argc == 2)
    resolution = atof(argv[1]);
  else
    {
      std::cerr << "Provide the octree resolution as an argument!" << std::endl;
      return -1;
    }

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

  std::cout << "Replacing this many points:" << newPointIdxVector.size() << std::endl;

  // Replace foreground points with background points
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

  pcl::io::savePCDFileASCII (path + "test_foreground_replaced.pcd", *cloud_fore);

}
