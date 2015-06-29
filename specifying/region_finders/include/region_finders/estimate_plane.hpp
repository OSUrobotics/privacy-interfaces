#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>


Eigen::Vector4f estimate_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
			       pcl::PointXYZRGB point)
{
  // Create the normal estimation class
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  //ne.setInputCloud (cloud);

  // Find the k-nearest-neighbors to the chosen point
  pcl::search::KdTree<pcl::PointXYZRGB> tree;
  tree.setInputCloud(cloud);
  double radius = 0.025;  // radius for k-nearest-neighbors search
  std::vector<int> indices;
  std::vector<float> distances;
  tree.radiusSearch(point, radius, indices, distances);
  std::cout << "Got " << indices.size() << " neighbors for plane estimation." << std::endl;

  // Fit a plane to just the k-nearest-neighbors to the clicked point
  Eigen::Vector4f plane_parameters;
  float curvature;
  ne.computePointNormal (*cloud, indices, plane_parameters, curvature);

  return plane_parameters;
}
