#include "ceiling_mapper.h"

CornerFinder::CornerFinder() : nh() {}

CornerFinder::~CornerFinder() {}

void CornerFinder::init() 
{
    // Create Subscribers
    std::cout << "Subscribing to Point Cloud" << std::endl;
    sub = nh.subscribe ("/camera/depth/points", 1, &CornerFinder::cloud_cb, this);
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

bool CornerFinder::is_paralell_planes(float a1, float b1, float c1, float a2, float b2, float c2)
{
    float threshold = 0.07;
    return (((b1/a1 + threshold) > b2/a2) && ((b1/a1 - threshold) < b2/a2)) &&
           (((c1/a1 + threshold) > c2/a2) && ((c1/a1 - threshold) < c2/a2));
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

void CornerFinder::find_corners(arma::fmat all_coefficients, arma::fmat all_intersections)
{
    arma::fmat plane_coefficents = arma::randu<arma::fmat>(3, 3);
    arma::fmat plane_intersections = arma::randu<arma::fmat>(3, 1);

    std::cout << "Finding corners from " << MAX_PLANES << " planes" << std::endl;

    for(int i=0; i<(MAX_PLANES-2); i++) 
    {
        if (isnan(all_intersections(i, 0)))
        {
            std::cout << "[i= " << i << "Plane " << i << " is not a plane" << std::endl;
            break;
        }
        std::cout << "[i=" << i << "] Putting plane " << i << " into coefficients matrix" << std::endl;
        plane_coefficents(0, 0) = all_coefficients(i, 0);
        plane_coefficents(0, 1) = all_coefficients(i, 1);
        plane_coefficents(0, 2) = all_coefficients(i, 2);
        plane_intersections(0, 0) = all_intersections(i, 0);
        for (int j=1; j<(MAX_PLANES-1); j++) 
        {
            if (isnan(all_intersections(j, 0))) 
            {
                std::cout << "[i=" << i << " j=" << j << "] Plane " << j << " is not a plane" << std::endl;
                break;
            }
            if (i == j) {}
            else if (is_paralell_planes(plane_coefficents(0, 0), 
                                  plane_coefficents(0, 1),
                                  plane_coefficents(0, 2), 
                                  all_coefficients(j, 0),
                                  all_coefficients(j, 1),
                                  all_coefficients(j, 2)))
            {
                std::cout << "[i=" << i << " j=" << j << "] Plane " << j << " is parallel to plane " << i << std::endl;
            } 
            else 
            {
                std::cout << "[i=" << i << " j=" << j << "] Putting plane " << j << " into coefficients matrix" << std::endl;
                plane_coefficents(1, 0) = all_coefficients(j, 0);
                plane_coefficents(1, 1) = all_coefficients(j, 1);
                plane_coefficents(1, 2) = all_coefficients(j, 2);
                plane_intersections(1, 0) = all_intersections(j, 0);
                for (int k=2; k<MAX_PLANES; k++) 
                {
                    if (isnan(all_intersections(k, 0)))
                    {
                        std::cout << "[i=" << i << " j=" << j << " k=" << k << "] Plane " << k << " is not a plane" << std::endl;
                        break;
                    }
                    if (j == i || j == k) {}
                    else if (is_paralell_planes(plane_coefficents(0, 0),
                                           plane_coefficents(0, 1),
                                           plane_coefficents(0, 2),
                                           all_coefficients(k, 0),
                                           all_coefficients(k, 1),
                                           all_coefficients(k, 2)))
                    {
                        std::cout << "[i=" << i << " j=" << j << " k=" << k << "] Plane " << k << " is parallel to plane " << i << std::endl;
                    }
                    else if (is_paralell_planes(plane_coefficents(1, 0),
                                           plane_coefficents(1, 1),
                                           plane_coefficents(1, 2),
                                           all_coefficients(k, 0),
                                           all_coefficients(k, 1),
                                           all_coefficients(k, 2)))
                    {
                        std::cout << "[i=" << i << " j=" << j << " k=" << k << "] Plane " << k << "is parallel to plane " << j << std::endl;
                    }
                    else 
                    {
                        std::cout << "[i=" << i << " j=" << j << " k=" << k << "] Putting plane " << k << "into coefficients matrix" << std::endl;
                        plane_coefficents(2, 0) = all_coefficients(k, 0);
                        plane_coefficents(2, 1) = all_coefficients(k, 1);
                        plane_coefficents(2, 2) = all_coefficients(k, 2);
                        plane_intersections(2, 0) = all_intersections(k, 0);
                        
                        std::cout << "Finding corner between planes " << i << ", " << j << ", " << k << std::endl;
                        publish_corner(&plane_coefficents, &plane_intersections);
                    }
                }
            }
        }
    }
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
    // Set up the segementation object with a distance threshold of 0.01
    set_segmentation_params(0.01, &seg);

    // Create floating point matricies to hold the coeficients
    arma::fmat plane_coefficients = arma::randu<arma::fmat>(MAX_PLANES, 3);
    arma::fmat plane_intersections = arma::randu<arma::fmat>(MAX_PLANES, 1);

    // Find the best planes
    for (int i=0; i<MAX_PLANES; i++)
    {
        if (cloud.size() > 0) 
        {
            // Set up the segmentation object
            seg.setInputCloud (cloud.makeShared());
            // Run the segmentation
            seg.segment(*inliers, coefficients); 
        }

        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            // The coefficients remain the same
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
        // publish_plane(&plane, i);
    }

    // find and publish the corner
    // publish_corner(&plane_coefficients, &plane_intersections);
    find_corners(plane_coefficients, plane_intersections);

    std::cout << std::endl;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    std::cout << "Intitializing ROS" << std::endl; 
    ros::init (argc, argv, "corner_finder");
    CornerFinder corner_finder;
    corner_finder.init();
    std::cout << "Ready to Recive messages" << std::endl;
    ros::spin ();
}