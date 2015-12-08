#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"

namespace aero
{
  namespace perception
  {

    class PlaneDetectedPointCloud : public PointCloudSensor
    {
    public: explicit PlaneDetectedPointCloud(ros::NodeHandle _nh);

    public: ~PlaneDetectedPointCloud();

    protected: void SubscribePoints(
	const sensor_msgs::PointCloud2::ConstPtr& _msg);

    protected: ros::Subscriber point_cloud_listener_;

    protected: Eigen::Vector3f desk_plane_norm_;

    protected: float height_range_per_region_;

    protected: ros::Publisher pcl_pub_;
    };

    typedef std::shared_ptr<PlaneDetectedPointCloud> PlaneDetectedPointCloudPtr;

  }
}

using namespace aero;
using namespace perception;

//////////////////////////////////////////////////
PlaneDetectedPointCloud::PlaneDetectedPointCloud(ros::NodeHandle _nh)
    : PointCloudSensor(_nh)
{
  height_range_per_region_ = 0.05;

  desk_plane_norm_ =
      Eigen::Quaternionf(1, 0, 0, 0) * Eigen::Vector3f(0, 0, 1);

  space_min_ = {-0.5, -0.5, 0.8};
  space_max_ = {0.5, 1.0, 5.0};

  point_cloud_listener_ =
      nh_.subscribe("/stereo/points2", 1000,
		    &PlaneDetectedPointCloud::SubscribePoints, this);

  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/visualized_object_pcl", 100);
}

//////////////////////////////////////////////////
PlaneDetectedPointCloud::~PlaneDetectedPointCloud()
{
}

//////////////////////////////////////////////////
void PlaneDetectedPointCloud::SubscribePoints(
    const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  Eigen::Vector3f manipulation_space_max(
      space_max_.x, space_max_.y, space_max_.z);
  Eigen::Vector3f manipulation_space_min(
      space_min_.x, space_min_.y, space_min_.z);

  float height_range_total =
    (manipulation_space_max - manipulation_space_min).norm();
  int num_of_regions =
    height_range_total / height_range_per_region_ + 1;
  float lowest_height_level = -height_range_total;

  static tf::TransformListener tl;
  tf::StampedTransform base_to_eye;
  Eigen::Quaternionf base_to_eye_q;
  ros::Time now = ros::Time::now();
  tl.waitForTransform("leg_base_link", "ps4eye_frame",
		      now, ros::Duration(2.0));

  try
  {
    tl.lookupTransform("leg_base_link", "ps4eye_frame", now, base_to_eye);
  }
  catch (std::exception e)
  {
    ROS_ERROR("failed tf listen");
    return;
  }

  base_to_eye_q =
    Eigen::Quaternionf(base_to_eye.getRotation().w(),
		       base_to_eye.getRotation().x(),
		       base_to_eye.getRotation().y(),
		       base_to_eye.getRotation().z());
  desk_plane_norm_ =
    base_to_eye_q.inverse() * Eigen::Vector3f(0, 0, -1);

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl, *raw);

  std::vector<std::vector<Eigen::Vector3f> > vertices_block(num_of_regions);
  std::vector<std::vector<Eigen::Vector3f> > vertices_shifted(num_of_regions);
  std::vector<int> elements_in_block(num_of_regions, 0);

  for (unsigned int i = 0; i < raw->points.size(); ++i)
    if ((raw->points[i].x > manipulation_space_min.x()) &&
	(raw->points[i].x < manipulation_space_max.x()) &&
	(raw->points[i].y > manipulation_space_min.y()) &&
	(raw->points[i].y < manipulation_space_max.y()) &&
	(raw->points[i].z > manipulation_space_min.z()) &&
	(raw->points[i].z < manipulation_space_max.z()))
    {
      Eigen::Vector3f point(
	  raw->points[i].x, raw->points[i].y, raw->points[i].z);
      int index =
          static_cast<int>(
	      ((manipulation_space_min - point).dot(desk_plane_norm_) -
	       lowest_height_level) / height_range_per_region_);
      int index_shifted =
          static_cast<int>(
	      ((manipulation_space_min - point).dot(desk_plane_norm_) -
	       lowest_height_level) / height_range_per_region_ - 0.5);
      if (index_shifted < 0) index_shifted = 0;
      Eigen::Vector3f tmp = manipulation_space_min - point;
      vertices_block[index].push_back(point);
      vertices_shifted[index_shifted].push_back(point);
    }

  // find table (continuous non-zero area)

  int start_from = 0;
  for (unsigned int i = 0; i < num_of_regions; ++i)
  {
    if (vertices_block[i].size() == 0) continue;

    int region_begin = i;
    while (vertices_block[i].size() > 0 && i < num_of_regions)
      ++i;
    if ((i - region_begin) * height_range_per_region_ > 0.5) // 50 cm
      // reject floor
      start_from = region_begin + (i - region_begin) * 0.5;
  }

  // find area with most points in highest plane (eliminate floor)

  int max_index = -1;
  int max_points = 0;
  int max_index_when_shifted = -1;
  int max_points_when_shifted = 0;
  // for (int i = plane_idx[plane_idx.size()-1]; i < num_of_regions; ++i)
  for (unsigned int i = start_from; i < num_of_regions; ++i)
  {
    if (vertices_block[i].size() == 0) continue;

    ROS_WARN("block %d has %d points", i, vertices_block[i].size());
    if (vertices_block[i].size() > max_points)
    {
      max_points = vertices_block[i].size();
      max_index = i;
    }
    if (vertices_shifted[i].size() > max_points_when_shifted)
    {
      max_points_when_shifted = vertices_shifted[i].size();
      max_index_when_shifted = i;
    }
  }

  // get the better region cut

  std::vector<Eigen::Vector3f> plane_points;
  if (max_points_when_shifted > max_points)
    plane_points.assign(vertices_shifted[max_index_when_shifted].begin(),
			vertices_shifted[max_index_when_shifted].end());
  else
    plane_points.assign(vertices_block[max_index].begin(),
			vertices_block[max_index].end());

  // now we have the points on the plane

  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->points.reserve(plane_points.size());

  for (unsigned int i = 0; i < plane_points.size(); ++i)
  {
    cloud_->points.push_back(pcl::PointXYZ(plane_points[i].x(),
					   plane_points[i].y(),
					   plane_points[i].z()));
  }

  pcl::PCLPointCloud2 pcl_out;
  sensor_msgs::PointCloud2 msg;
  pcl::toPCLPointCloud2(*cloud_, pcl_out);
  pcl_conversions::fromPCL(pcl_out, msg);
  msg.header.frame_id = "ps4eye_frame";
  msg.header.stamp = ros::Time(0); // get possible recent
  pcl_pub_.publish(msg);
}


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation");
  ros::NodeHandle nh;

  aero::perception::PlaneDetectedPointCloudPtr sensor(
      new aero::perception::PlaneDetectedPointCloud(nh));

  ros::spin();

  return 0;
}
