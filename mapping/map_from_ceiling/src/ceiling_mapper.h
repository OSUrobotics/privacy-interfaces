#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/PointStamped.h>
#include <math.h>
// For linear algebra. Compile with -O1 -larmadillo
#include <armadillo>

#define MAX_PLANES 5

class Corner : public geometry_msgs::PointStamped
{
public:
    Corner();
    ~Corner();
private:
    int plane_1_id, plane_2_id, plane_3_id;
};

class CornerFinder
{
public:
    CornerFinder();
    ~CornerFinder();

    void init();
    void spin();

private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub, pub_plane_1, pub_plane_2, pub_plane_3;

    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);
    void set_segmentation_params(float distance_threshold, pcl::SACSegmentation<pcl::PointXYZ>* seg);
    void print_coefficients(pcl::ModelCoefficients coefficients, int plane_number);
    void extract_inliers(pcl::PointCloud<pcl::PointXYZ>* original_pc, pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>* plane);
    void publish_plane(pcl::PointCloud<pcl::PointXYZ>* to_publish, int publisher_index);
    void publish_corner(arma::fmat* plane_coefficients, arma::fmat* plane_intersections);
    void find_corners(arma::fmat plane_coefficients, arma::fmat plane_intersections);
    bool is_paralell_planes(float a1, float b1, float c1, float a2, float b2, float c2);
};