#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


pcl::PointCloud<pcl::PointXYZRGB>::Ptr extract_clusters (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB point)
{
  cloud->push_back(point);  // append clicked_point to cloud
  int point_index = cloud->points.size() - 1;  // get index of clicked_point

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (640*480);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	  
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {

      if(std::find(it->indices.begin(), it->indices.end(), point_index) != it->indices.end())
	{
	  // Pointer-ify the indices for the current cluster
	  pcl::PointIndicesPtr indices_ptr (new pcl::PointIndices);
	  indices_ptr->indices = it->indices; 
	  
	  // Extract the cluster points
	  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	  extract.setInputCloud (cloud);
	  extract.setIndices (indices_ptr);
	  extract.setNegative (false);
	  extract.filter (*cloud_cluster);
	  
	  std::cout << "PointCloud representing cluster #" << j << ": " << cloud_cluster->points.size () << " data points." << std::endl;
	  j++;
	}
    }

  return cloud_cluster;
}
