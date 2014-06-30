#include "ceiling_mapper.h"

ros::Publisher pub;
ros::Publisher pub_plane_1;
ros::Publisher pub_plane_2;
ros::Publisher pub_plane_3;

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
    // Create the Filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);  

    // Create floating point matricies to hold the coeficients
    arma::fmat plane_coefficients = arma::randu<arma::fmat>(3, 3);
    arma::fmat plane_intersections = arma::randu<arma::fmat>(3, 1);

    // Find the best three planes
    for (int i=0; i<3; i++)
    {
        // Set up the segmentation object
        seg.setInputCloud (cloud.makeShared ());
        // Run the segmentation
        seg.segment (*inliers, coefficients); 
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            return;
        }

        // convert the coeficents to a ROS message
        pcl_msgs::ModelCoefficients ros_coefficients;
        pcl_conversions::fromPCL(coefficients, ros_coefficients);
      
        std::cout << "[" << i << "] Found a plane with Coefficients: " 
                              << coefficients.values[0] << " "
                              << coefficients.values[1] << " "
                              << coefficients.values[2] << " "
                              << coefficients.values[3] << std::endl;   

        // throw the coefficients in the coefficents matrix
        for (int j=0; j<3; j++)  
        {
            plane_coefficients(i, j) = coefficients.values[j];
        } 
        plane_intersections(i, 0) = -coefficients.values[3];

        std::cout << "Removing Inliners" << std::endl;
        pcl::PointCloud<pcl::PointXYZ> cloud_no_plane = cloud;
        extract.setInputCloud(cloud.makeShared());
        extract.setIndices(inliers);
        extract.setNegative(false);
        // Extract the plane from the point cloud
        extract.filter(plane);
        std::cout << "Plane has " << plane.width * plane.height 
                                  << " data points" << std::endl;
        extract.setNegative(true);
        extract.filter(cloud_no_plane);
        cloud.swap(cloud_no_plane);    

        sensor_msgs::PointCloud2 message;    

        pcl::toROSMsg(plane, message);    

        switch(i) {
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

    arma::fmat corner = solve(plane_coefficients, plane_intersections);

    corner.print("Corner: ");

    geometry_msgs::PointStamped corner_point;
    corner_point.header.seq = 0;
    corner_point.header.stamp = ros::Time::now();
    corner_point.header.frame_id = "/camera_rgb_optical_frame";
    corner_point.point.x = corner(0);
    corner_point.point.y = corner(1);
    corner_point.point.z = corner(2);

    pub.publish(corner_point);

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