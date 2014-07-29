// Synchronizes rgb and depth images to skeleton msgs using
// ApproximateTimePolicy from message_filters.

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <person_filter/SkeletonArray.h>
#include <std_msgs/Header.h>

using namespace sensor_msgs;
using namespace person_filter;
using namespace message_filters;

ros::Publisher pub_rgb, pub_depth, pub_skeletons;

void callback(const ImageConstPtr& image_rgb, const ImageConstPtr& image_depth, const SkeletonArrayConstPtr& skeletons)
{
  // Copy RGB image -- HACK!
  Image image_rgb_sync;
  image_rgb_sync.header = image_rgb->header;
  image_rgb_sync.height = image_rgb->height;
  image_rgb_sync.width = image_rgb->width;
  image_rgb_sync.encoding = image_rgb->encoding;
  image_rgb_sync.is_bigendian = image_rgb->is_bigendian;
  image_rgb_sync.step = image_rgb->step;
  image_rgb_sync.data = image_rgb->data;

  // Copy Depth image -- HACK!
  Image image_depth_sync;
  image_depth_sync.header = image_depth->header;
  image_depth_sync.height = image_depth->height;
  image_depth_sync.width = image_depth->width;
  image_depth_sync.encoding = image_depth->encoding;
  image_depth_sync.is_bigendian = image_depth->is_bigendian;
  image_depth_sync.step = image_depth->step;
  image_depth_sync.data = image_depth->data;

  // Copy Skeleton array -- HACK!
  SkeletonArray skeletons_sync;
  skeletons_sync.header = skeletons->header;
  skeletons_sync.skeletons = skeletons->skeletons;

  // Make timestamps equal!
  ros::Time stamp = skeletons->header.stamp;
  image_rgb_sync.header.stamp = stamp;
  image_depth_sync.header.stamp = stamp;

  // Publish
  pub_rgb.publish(image_rgb_sync);
  pub_depth.publish(image_depth_sync);
  pub_skeletons.publish(skeletons_sync);

  ROS_INFO("Callback!");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synchronizer");

  ros::NodeHandle nh;

  // Initialize publishers
  pub_rgb = nh.advertise <Image> ("/camera/rgb/image_color/sync", 1);
  pub_depth = nh.advertise <Image> ("/camera/depth_registered/image_raw/sync", 1);
  pub_skeletons = nh.advertise <SkeletonArray> ("/skeletons_uv/sync", 1);

  // Initialize subscribers
  message_filters::Subscriber<Image> image_rgb_sub(nh, "/camera/rgb/image_color", 1);
  message_filters::Subscriber<Image> image_depth_sub(nh, "/camera/depth_registered/image_raw", 1);
  message_filters::Subscriber<SkeletonArray> skeletons_sub(nh, "/skeletons_uv", 1);

  typedef sync_policies::ApproximateTime<Image, Image, SkeletonArray> MySyncPolicy;

  Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), image_rgb_sub, image_depth_sub, skeletons_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}
