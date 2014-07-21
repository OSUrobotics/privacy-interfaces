// Uses VoxelGrid filter to downsample PointCloud2 messages.

#include <iostream>
#include <stdlib.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>  // allows subscribing/publishing PCL types as ROS msgs

// PCL
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


// Globals :-(
float voxel_size = 0.02;  // 2cm default voxel size
ros::Publisher pub;


//void cloud_callback (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
void cloud_callback (pcl::PCLPointCloud2::Ptr cloud_in)
{
  // Initial cloud size
  std::cerr << "PointCloud before filtering: " << cloud_in->width * cloud_in->height 
	    << " data points (" << pcl::getFieldsList (*cloud_in) << ")." << std::endl;

  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (voxel_size, voxel_size, voxel_size);
  sor.filter (*cloud_filtered);

  // Filtered cloud size
  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
	    << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

  pub.publish (cloud_filtered);
}


int main (int argc, char** argv)
{

  if (argc >= 2)
    voxel_size = atof(argv[1]);  // set voxel size

  ros::init(argc, argv, "downsample_clouds");
  ros::NodeHandle node;

  //  pub = node.advertise < pcl::PointCloud <pcl::PointXYZRGB> > ("cloud_out", 5);
  pub = node.advertise <pcl::PCLPointCloud2> ("cloud_out", 5);
  ros::Subscriber sub = node.subscribe("cloud_in", 5, cloud_callback);

  ros::spin();
}
