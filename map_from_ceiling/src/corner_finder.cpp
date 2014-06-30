#include "ceiling_mapper.h"

CornerFinder::CornerFinder() : nh() {}

CornerFinder::~CornerFinder() {}

void CornerFinder::init() 
{
    // Create Subscribers
    ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, &CornerFinder::cloud_cb, this);
    // Create Publishers
    pub = nh.advertise<geometry_msgs::PointStamped> ("/output", 1);
    pub_plane_1 = nh.advertise<sensor_msgs::PointCloud2> ("/Plane_1", 1);
    pub_plane_2 = nh.advertise<sensor_msgs::PointCloud2> ("/Plane_2", 1);
    pub_plane_3 = nh.advertise<sensor_msgs::PointCloud2> ("/Plane_3", 1);
}

void CornerFinder::set_segmentation_params(float distance_threshold, pcl::SACSegmentation<pcl::PointXYZ>* seg)
{
    // Optional
    seg->setOptimizeCoefficients (true);
    // Mandatory
    seg->setModelType (pcl::SACMODEL_PLANE);
    seg->setMethodType (pcl::SAC_RANSAC);
    seg->setDistanceThreshold (distance_threshold);  
}

void CornerFinder::print_coefficients(pcl::ModelCoefficients coefficients, int plane_number)
{
    std::cout << "[" << plane_number << "] Found a plane with Coefficients: " 
                                     << coefficients.values[0] << " "
                                     << coefficients.values[1] << " "
                                     << coefficients.values[2] << " "
                                     << coefficients.values[3] << std::endl;   
}

void CornerFinder::extract_inliers(pcl::PointCloud<pcl::PointXYZ>* original_pc, pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZ>* plane) 
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

void CornerFinder::publish_plane(pcl::PointCloud<pcl::PointXYZ>* to_publish, int publisher_index) 
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

void CornerFinder::publish_corner(arma::fmat* plane_coefficients, arma::fmat* plane_intersections)
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

void CornerFinder::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
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
    CornerFinder corner_finder;
    corner_finder.init();

    ros::spin ();
}