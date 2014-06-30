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
// For linear algebra. Compile with -O1 -larmadillo
#include <armadillo>

ros::Publisher pub;
ros::Publisher pub_plane_1;
ros::Publisher pub_plane_2;
ros::Publisher pub_plane_3;

void set_segmentation_params(float distance_threshold, pcl::SACSegmentation<pcl::PointXYZ>* seg)
{
    // Optional
    seg->setOptimizeCoefficients (true);
    // Mandatory
    seg->setModelType (pcl::SACMODEL_PLANE);
    seg->setMethodType (pcl::SAC_RANSAC);
    seg->setDistanceThreshold (distance_threshold);  
}

void print_coefficients(pcl::ModelCoefficients coefficients, int plane_number)
{
    std::cout << "[" << plane_number << "] Found a plane with Coefficients: " 
                                     << coefficients.values[0] << " "
                                     << coefficients.values[1] << " "
                                     << coefficients.values[2] << " "
                                     << coefficients.values[3] << std::endl;   
}

void extract_inliers(pcl::PointCloud<pcl::PointXYZ>* original_pc, pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>* plane) 
{
    // Create the Filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    std::cout << "Removing Inliners" << std::endl;
    pcl::PointCloud<pcl::PointXYZ> cloud_no_plane = *original_pc;
    extract.setInputCloud(original_pc->makeShared());
    extract.setIndices(inliers);
    extract.setNegative(false);
    // Extract the plane from the point cloud
    extract.filter(*plane);
    std::cout << "Plane has " << plane->width * plane->height 
                              << " data points" << std::endl;
    extract.setNegative(true);
    extract.filter(cloud_no_plane);
    original_pc->swap(cloud_no_plane);
}

void publish_plane(pcl::PointCloud<pcl::PointXYZ>* to_publish, int publisher_index) 
{
    sensor_msgs::PointCloud2 message;    

    pcl::toROSMsg(*to_publish, message);    

    switch(publisher_index) 
    {
        case 0: 
            pub_plane_1.publish(message); 
            break;
        case 1:
            pub_plane_2.publish(message);
            break;
        case 2:
            pub_plane_3.publish(message);
    }
}

void publish_corner(arma::fmat* plane_coefficients, arma::fmat* plane_intersections)
{
    arma::fmat corner = solve(*plane_coefficients, *plane_intersections);

    corner.print("Corner: ");

    geometry_msgs::PointStamped corner_point;
    corner_point.header.seq = 0;
    corner_point.header.stamp = ros::Time::now();
    corner_point.header.frame_id = "/camera_rgb_optical_frame";
    corner_point.point.x = corner(0);
    corner_point.point.y = corner(1);
    corner_point.point.z = corner(2);

    pub.publish(corner_point);
}

class Corner 
{

};

class CornerFinder
{
public:
    CornerFinder();
    ~CornersFinder();

    void init();
    void spin();

private:
    ros::NodeHandle nh;

};