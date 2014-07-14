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
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/common/centroid.h>

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
pcl::visualization::ImageViewer viewer_highlight ("Highlighted foreground objects");
pcl::visualization::ImageViewer viewer_replaced ("Foreground? What foreground?");
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_show;
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


void color_indices(int color,
 		   std::vector<int> indices,
		   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore)
{
  // Parse 'color' input (0 = red, 1 = green, 2 = blue)
  int r, g, b;
  r = g = b = 0;
  if (color == 0)
    r = 1;
  else if (color == 1)
    g = 1;
  else
    b = 1;
  
  // Replace indexed points with color 
  std::cout << "Coloring this many points:" << indices.size() << std::endl;
  for (size_t i = 0; i < indices.size (); ++i)
    {
      cloud_fore->points[indices[i]].r *= r;
      cloud_fore->points[indices[i]].g *= g;
      cloud_fore->points[indices[i]].b *= b;
    }

  std::cout << "Colored those points!" << std::endl;
}


std::vector<int> filter_foreground (float resolution,
			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_back, 
			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore)
{
  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree (resolution);

  // Add points from cloud_back_xyz to octree
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

std::vector<pcl::PointIndices> cluster_indices (std::vector<int> indices_fore,
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fore)
{
  // Get cloud of foreground only
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_fore (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::ExtractIndices<pcl::PointXYZRGB> ex (false);
  ex.setKeepOrganized (false);
  //ex.setUserFilterValue(0.00f);
  ex.setInputCloud (cloud_fore);
  pcl::PointIndices::Ptr indices_fore_ptr (new pcl::PointIndices ());
  indices_fore_ptr->indices = indices_fore;
  ex.setIndices (indices_fore_ptr);
  ex.filter (*points_fore);
  //viewer_highlight.showRGBImage(*points_fore);
  std::cout << "Points in foreground: " << points_fore->width * points_fore->height << std::endl;

  // Octree-based filter because DANG that's a lot of points to cluster
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxels_fore (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::octree::OctreePointCloudPointVector<pcl::PointXYZRGB> octree_fore (0.05f); 
  octree_fore.setInputCloud(points_fore); 
  octree_fore.addPointsFromInputCloud(); 
  

  pcl::octree::OctreePointCloudPointVector<pcl::PointXYZRGB>::LeafNodeIterator it (&octree_fore);
  std::vector<pcl::PointIndices> indices_vox;
  pcl::PointIndices index_vox;  // temp
  Eigen::Vector4f centroid;  // temp
  pcl::PointXYZ point;  // temp
  while (*++it) 
    {
      //      std::cout << (*it)->getNodeType() << " ";
      //      std::cout << it.getLeafContainer().getPointIndicesVector().size() << std::endl;
      index_vox.indices = it.getLeafContainer().getPointIndicesVector();
      indices_vox.push_back (index_vox);

      pcl::compute3DCentroid (*points_fore, index_vox, centroid);
      point.getVector4fMap() = centroid;
      voxels_fore->push_back (point);
    }
  std::cout << "Foreground points after downsampling: " << voxels_fore->width * voxels_fore->height << std::endl;
  std::cout << "Foreground indices after downsampling: " << indices_vox.size() << std::endl;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (voxels_fore);

  // Cluster it!
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.05); // about two inches
  //ec.setMinClusterSize (100);
  //ec.setMaxClusterSize (480*640/2);
  ec.setSearchMethod (tree);
  ec.setInputCloud (voxels_fore);
  ec.extract (clusters);

  std::cout << "Number of clusters: " << clusters.size() << std::endl;
  std::cout << "Taking largest cluster from now on." << std::endl;

  // Make indices refer back to the full point cloud
  pcl::PointIndices cluster;  // temp
  std::cout << "Expanding voxels by cluster..." << std::endl;
  for (std::vector<pcl::PointIndices>::iterator it = clusters.begin (); it != clusters.end (); ++it)  // for each cluster
    {
      for (std::vector<int>::iterator it2 = it->indices.begin (); it2 != it->indices.end (); ++it2)  // for each voxel in this cluster
	for (std::vector<int>::iterator it3 = indices_vox[*it2].indices.begin (); it3 != indices_vox[*it2].indices.end (); ++it3)  // for each point in this voxel
	    cluster.indices.push_back (indices_fore[*it3]);

      *it = cluster;  // update cluster
      cluster.indices.clear();  // clear our temp cluster
    }
  std::cout << "Largest cluster size: " << clusters[0].indices.size() << std::endl;

  return clusters;
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
  else
    {
      cloud_show = cloud_back->makeShared();
      
      std::vector<int> indices_fore = filter_foreground(resolution, cloud_back, cloud_fore);
      std::vector<int> indices_nan = filter_nans(cloud_back);  // of BACKGROUND image!

      std::vector<pcl::PointIndices> clusters = cluster_indices(indices_fore, cloud_fore);

      // Project biggest cluster onto background image
      replace_indices(clusters[0].indices, cloud_fore, cloud_show);
      //replace_indices(indices_nan, cloud_back, cloud_fore);  // replace the NaNs, too
      if (!viewer_replaced.wasStopped())
      	viewer_replaced.showRGBImage(*cloud_show);

      // Highlight what we've kept
      color_indices(0, clusters[0].indices, cloud_fore);
      if (!viewer_highlight.wasStopped())
        viewer_highlight.showRGBImage(*cloud_fore);
    }

  std::cout << std::endl;  // for sanity's sake
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
