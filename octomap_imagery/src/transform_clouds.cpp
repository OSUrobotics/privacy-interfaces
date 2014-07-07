// Node to take PointCloud2 msgs and transform them into another TF frame.

#include <string>

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <pcl_conversions/pcl_conversions.h>

using namespace ros;
using namespace pcl;

tf::TransformListener listener;
std::string frame_new;
Publisher pub;

void transform_cloud(sensor_msgs::PointCloud2 cloud_in) 
{
  bool can_transform = listener.waitForTransform(cloud_in.header.frame_id, 
						 frame_new,
						 ros::Time(0), 
						 ros::Duration(3.0));
  if (can_transform)
    {
      sensor_msgs::PointCloud2 cloud_out;
      cloud_in.header.stamp = ros::Time(0);
      pcl_ros::transformPointCloud(frame_new, cloud_in, cloud_out, listener);
      pub.publish (cloud_out);
    }
}


int main(int argc, char **argv) 
{
  if (argc >= 2)
    frame_new = argv[1]; 
  else
    return -1;

  init(argc, argv, "transform_clouds");
  NodeHandle node;

  pub = node.advertise <sensor_msgs::PointCloud2> ("cloud_out", 5);
  Subscriber sub = node.subscribe("cloud_in", 5, transform_cloud);

  spin();
}
