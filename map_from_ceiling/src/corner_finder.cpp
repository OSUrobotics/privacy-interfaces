#include "ceiling_mapper.h"

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, cloud);

    // For storing the newly found plane
    pcl::PointCloud<pcl::PointXYZ> plane;

    std::cout << "[" << ros::Time::now() << "] Received PointCloud2 Message with " 
      << cloud.width * cloud.height << " data points" << std::endl;

    // Create the coefficient, inliers objects
    pcl::ModelCoefficients coefficients;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Set up the segementation object with a distance threshold of 0.1
    set_segmentation_params(0.1, &seg);

    // Create floating point matricies to hold the coeficients
    arma::fmat plane_coefficients = arma::randu<arma::fmat>(3, 3);
    arma::fmat plane_intersections = arma::randu<arma::fmat>(3, 1);

    // Find the best three planes
    for (int i=0; i<3; i++)
    {
        // Set up the segmentation object
        seg.setInputCloud (cloud.makeShared());
        // Run the segmentation
        seg.segment(*inliers, coefficients); 

        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            return;
        }

        // convert the coeficents to a ROS message
        pcl_msgs::ModelCoefficients ros_coefficients;
        pcl_conversions::fromPCL(coefficients, ros_coefficients);
        print_coefficients(coefficients, i);

        // throw the coefficients in the coefficents matrix
        for (int j=0; j<3; j++)  
        {
            plane_coefficients(i, j) = coefficients.values[j];
        } 
        plane_intersections(i, 0) = -coefficients.values[3];

        extract_inliers(&cloud, inliers, &plane);
        publish_plane(&plane, i);
    }

    // find and publish the corner
    publish_corner(&plane_coefficients, &plane_intersections);

    std::cout << std::endl;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    std::cout << "Intitializing ROS" << std::endl; 
    ros::init (argc, argv, "corner_finder");
    ros::NodeHandle nh;  

    // Create a ROS subscriber for the input point cloud
    std::cout << "Subscribing to Point Cloud" << std::endl;
    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_cb);  

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<geometry_msgs::PointStamped> ("/output", 1);
    pub_plane_1 = nh.advertise<sensor_msgs::PointCloud2> ("/Plane_1", 1);
    pub_plane_2 = nh.advertise<sensor_msgs::PointCloud2> ("/Plane_2", 1);
    pub_plane_3 = nh.advertise<sensor_msgs::PointCloud2> ("/Plane_3", 1);

    std::cout << "Ready to Receive Messages" << std::endl;
    // Spin
    ros::spin ();
}