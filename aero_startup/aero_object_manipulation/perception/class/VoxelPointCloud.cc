#include "aero_object_manipulation/perception/class/VoxelPointCloud.hh"

using namespace aero;
using namespace perception;

//////////////////////////////////////////////////
VoxelPointCloud::VoxelPointCloud(ros::NodeHandle _nh) : PointCloudSensor(_nh)
{
 point_cloud_listener_ =
    nh_.subscribe("/stereo/points2", 1000,
  		  &VoxelPointCloud::SubscribePoints, this);
  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/stereo/points2/compressed", 100);
}

//////////////////////////////////////////////////
VoxelPointCloud::~VoxelPointCloud()
{
}

//////////////////////////////////////////////////
void VoxelPointCloud::SubscribePoints(
    const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  pcl::PCLPointCloud2::Ptr pcl(new pcl::PCLPointCloud2());
  pcl_conversions::toPCL(*_msg, *pcl);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(pcl);
  sor.setLeafSize(0.02f, 0.02f, 0.02f);
  sor.filter(cloud_filtered);
  sensor_msgs::PointCloud2 msg;
  pcl_conversions::fromPCL(cloud_filtered, msg);
  msg.header.frame_id = "ps4eye_frame";
  msg.header.stamp = ros::Time(0);
  pcl_pub_.publish(msg);
}
