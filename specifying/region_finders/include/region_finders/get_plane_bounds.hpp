#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>  // added
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

// From PCL tutorials: pointclouds.org/documentation/tutorials
#include <region_finders/extract_clusters.hpp>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_plane_bounds(Eigen::Vector4f plane, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB point, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull, std::vector<pcl::Vertices> &vertices)
// function parameters "cloud_hull" and "vertices" are outputs. Parameter "vertices" is passed by reference.
{
  // Downsample the dataset using a leaf size of 1cm  (OPTIONAL, but increases frame rate)
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.02f, 0.02f, 0.02f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Get plane inliers
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr sac (new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (cloud_filtered));
  //sac.setInputCloud(cloud);
  std::vector<int> inliers;
  double threshold = 0.10;  // 10cm
  sac -> selectWithinDistance(plane, threshold, inliers);
  pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_filtered, inliers, *cloud_inliers);
  std::cerr << "PointCloud after segmentation has: "
            << inliers.size() << " inliers." << std::endl;

  // Select cluster that includes the selected point
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster = extract_clusters(cloud_inliers, point);

  // Create a Convex Hull representation of the inliers
  pcl::ConvexHull<pcl::PointXYZRGB> chull;
  chull.setInputCloud (cloud_cluster);
  chull.reconstruct (*cloud_hull, vertices);

  std::cerr << "Convex hull has: " << cloud_hull->points.size ()
            << " data points." << std::endl;

  return cloud_hull;
}
