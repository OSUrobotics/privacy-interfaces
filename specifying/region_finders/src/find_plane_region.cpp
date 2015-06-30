// Input: 
//   a 3d point and a point cloud
// Outputs: 
//   estimated plane at that point
//   borders of that planar region
//   triangulized 3d convex hull bounding that region (for filtering)

// ROS imports
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>  // allows subscribing/publishing PCL types as ROS msgs

// From PCL tutorials: pointclouds.org/documentation/tutorials
#include <region_finders/estimate_plane.hpp>
#include <region_finders/get_plane_bounds.hpp>
#include <region_finders/triangulate_pointcloud.hpp>

// Globals
ros::Publisher pub_cloud;
pcl::PointXYZRGB clicked_point;
bool have_clicked_point = false;

void point_callback (geometry_msgs::PointStamped::Ptr point_in)
{
  if (!have_clicked_point)
    {
      std::cerr << "Got a clicked_point!" << std::endl;
      have_clicked_point = true;

      // Copy to pcl::PointXYZ type
      clicked_point.x = point_in->point.x;
      clicked_point.y = point_in->point.y;
      clicked_point.z = point_in->point.z;

      // Write clicked_point value to screen
      std::cerr << "Clicked point: " << clicked_point.x << "," 
		<< clicked_point.y << "," 
		<< clicked_point.z << std::endl;

    }
  else
    {
      std::cerr << "Keeping the old clicked point...restart node to change it." << std::endl;
    }
      
  return;
}

void cloud_callback (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
  if (!have_clicked_point)  // return early if no clicked point yet
    {
      return;
    }

  // Initial cloud size
  std::cerr << "\n" << "PointCloud initial size: " << cloud_in->width * cloud_in->height 
	    << " data points (" << pcl::getFieldsList (*cloud_in) << ")." << std::endl;

  // Get plane at clicked point
  Eigen::Vector4f plane = estimate_plane(cloud_in, clicked_point);
  std::cerr << "Plane about clicked point: " << plane[0] << "," 
	    << plane[1] << "," 
	    << plane[2] << "," 
	    << plane[3] << std::endl;

  // Get plane bounds
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bounds = get_plane_bounds(plane, cloud_in);

  // Triangulate them
  pcl::PolygonMesh triangles = triangulate_pointcloud(cloud_bounds);

  // Publish mesh of bounds as well as plane parameters
  pub_cloud.publish (cloud_bounds);
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "find_plane_region");
  ros::NodeHandle node;

  pub_cloud = node.advertise < pcl::PointCloud <pcl::PointXYZRGB> > ("cloud_out", 5);
  ros::Subscriber sub_cloud = node.subscribe("/camera/depth_registered/points", 5, cloud_callback);
  ros::Subscriber sub_point = node.subscribe("/clicked_point", 5, point_callback);

  ros::spin();
}

