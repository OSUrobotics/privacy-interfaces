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
#include <shape_msgs/Plane.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Point.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_tools/shape_to_marker.h>  // magic conversion functions

// From PCL tutorials: pointclouds.org/documentation/tutorials
#include <region_finders/estimate_plane.hpp>
#include <region_finders/get_plane_bounds.hpp>

// Globals
ros::Publisher pub_plane;
ros::Publisher pub_mesh;
ros::Publisher pub_marker;
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

  // Get plane bounds and mesh them
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bounds (new pcl::PointCloud<pcl::PointXYZRGB> ());
  std::vector<pcl::Vertices> vertices;  // for a meshed version of the hull
  get_plane_bounds(plane, cloud_in, clicked_point, cloud_bounds, vertices);

  // Convert to ROS msg types
  // 1) Eigen::Vector4f to shape_msgs::Plane::Ptr
  shape_msgs::Plane plane_msg;
  for (int i = 0; i < 4; i++)
    plane_msg.coef[i] = plane[i];
  // 2) pcl::PolygonMesh to shape_msgs::Mesh::Ptr
  shape_msgs::Mesh triangles_msg;
  geometry_msgs::Point vertex;
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator it_cloud = cloud_bounds->begin(); it_cloud < cloud_bounds->end(); it_cloud++)  // copy over the vertices themselves ((x,y,z) triples)
    {
      vertex.x = it_cloud->x;
      vertex.y = it_cloud->y;
      vertex.z = it_cloud->z;
      triangles_msg.vertices.push_back(vertex);
    }
  shape_msgs::MeshTriangle triangle;
  for (std::vector<pcl::Vertices>::iterator it_poly = vertices.begin(); it_poly < vertices.end(); it_poly++)  // copy over the triples of triangle indices, where each index refers to a vertex
    {
      for (int i = 0; i < 3; i++)
	triangle.vertex_indices[i] = it_poly->vertices[i];
      triangles_msg.triangles.push_back(triangle);
    }

  // Convert mesh to marker
  visualization_msgs::Marker marker;
  bool use_triangle_list = true;
  shape_tools::constructMarkerFromShape(triangles_msg, marker, use_triangle_list);
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/camera_rgb_optical_frame";
  marker.color.g = 1.0;
  marker.color.a = 0.5;

  // Publish mesh of bounds as well as plane parameters
  pub_plane.publish (plane_msg);
  pub_mesh.publish (triangles_msg);
  pub_marker.publish (marker);
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "find_plane_region");
  ros::NodeHandle node;

  pub_plane = node.advertise <shape_msgs::Plane> ("zones/planar/region_0/plane", 5);
  pub_mesh = node.advertise <shape_msgs::Mesh> ("zones/planar/region_0/mesh", 5);
  pub_marker = node.advertise <visualization_msgs::Marker> ("zones/planar/region_0/marker", 5);
  ros::Subscriber sub_cloud = node.subscribe("/camera/depth_registered/points", 5, cloud_callback);
  ros::Subscriber sub_point = node.subscribe("/clicked_point", 5, point_callback);

  ros::spin();
}

