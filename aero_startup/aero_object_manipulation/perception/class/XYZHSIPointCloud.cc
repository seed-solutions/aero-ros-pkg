#include "aero_object_manipulation/perception/class/XYZHSIPointCloud.hh"

using namespace aero;
using namespace perception;

//////////////////////////////////////////////////
XYZHSIPointCloud::XYZHSIPointCloud(ros::NodeHandle _nh) : PointCloudSensor(_nh)
{
  target_hsi_max_ = {0, 0, 0};
  target_hsi_min_ = {0, 0, 0};
  space_min_ = {-0.3, -0.3, 0.0};
  space_max_ = {0.3, 0.3, 1.0};
  point_cloud_listener_ = nh_.subscribe("/stereo/points2", 1000,
					&XYZHSIPointCloud::SubscribePoints, this);
  // pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/visualized_object_pcl", 100);
}

//////////////////////////////////////////////////
XYZHSIPointCloud::~XYZHSIPointCloud()
{
}

//////////////////////////////////////////////////
void XYZHSIPointCloud::SubscribePoints(const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl, *raw);

  Eigen::Vector3f raw_center(0, 0, 0);
  std::vector<Eigen::Vector3f> raw_vertices;
  std::vector<aero::rgb> raw_rgbs;
  raw_vertices.reserve(raw->points.size());
  raw_rgbs.reserve(raw->points.size());
  for (unsigned int i = 0; i < raw->points.size(); ++i)
    if ((raw->points[i].x > space_min_.x) &&
        (raw->points[i].x < space_max_.x) &&
        (raw->points[i].y > space_min_.y) &&
        (raw->points[i].y < space_max_.y) &&
        (raw->points[i].z > space_min_.z) &&
        (raw->points[i].z < space_max_.z))
    {
      rgb color = {raw->points[i].r, raw->points[i].g, raw->points[i].b};
      if (ValidHSI(color))
      {
	raw_vertices.push_back(
	    Eigen::Vector3f(raw->points[i].x, raw->points[i].y, raw->points[i].z));
	raw_rgbs.push_back(color);
	raw_center +=
	    Eigen::Vector3f(raw->points[i].x, raw->points[i].y, raw->points[i].z);
      }
    }

  raw_vertices.resize(raw_vertices.size());
  raw_rgbs.resize(raw_rgbs.size());
  raw_center = raw_center * (1.0 / raw_vertices.size());

  // ROS_INFO("found %d points", raw_vertices.size());
  // ROS_INFO("hsi : [%d ~ %d, %d ~ %d, %d ~ %d]",
  // 	   target_hsi_min.h, target_hsi_max.h, target_hsi_min.s, target_hsi_max.s,
  // 	   target_hsi_min.i, target_hsi_max.i);
  // ROS_INFO("xyz : [%f ~ %f, %f ~ %f, %f ~ %f]",
  // 	   space_min.x, space_max.x, space_min.y, space_max.y,
  // 	   space_min.z, space_max.z);

  Eigen::Vector3f raw_variance(0, 0, 0);
  std::vector<Eigen::Vector3f> raw_variances(raw_vertices.size());
  for (unsigned int i = 0; i < raw_vertices.size(); ++i)
  {
    raw_variances[i] = Eigen::Vector3f(
	 (raw_vertices[i][0] - raw_center[0]) * (raw_vertices[i][0] - raw_center[0]),
	 (raw_vertices[i][1] - raw_center[1]) * (raw_vertices[i][1] - raw_center[1]),
	 (raw_vertices[i][2] - raw_center[2]) * (raw_vertices[i][2] - raw_center[2]));
    raw_variance += raw_variances[i];
  }
  raw_variance = raw_variance * (4.0 / raw_vertices.size()); // cutting 5% of points

  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->points.reserve(raw_vertices.size());

  int vertices_count = 0;
  center_ = Eigen::Vector3f(0, 0, 0);
  vertices_.clear();
  vertices_.reserve(raw_vertices.size());
  rgb_.clear();
  rgb_.reserve(raw_rgbs.size());
  for (unsigned int i = 0; i < raw_vertices.size(); ++i)
    if ((raw_variances[i][0] < raw_variance[0]) &&
	(raw_variances[i][1] < raw_variance[1]) &&
	(raw_variances[i][2] < raw_variance[2]))
    {
      vertices_.push_back(raw_vertices[i]);
      rgb_.push_back(raw_rgbs[i]);
      center_ += raw_vertices[i];
      cloud_->points.push_back(pcl::PointXYZ(raw_vertices[i][0],
					     raw_vertices[i][1], raw_vertices[i][2]));
      ++vertices_count;
    }

  vertices_.resize(vertices_count);
  rgb_.resize(vertices_count);
  cloud_->points.resize(vertices_count);
  center_ = center_ * (1.0 / vertices_count);
}

//////////////////////////////////////////////////
bool XYZHSIPointCloud::ValidHSI(aero::rgb _color)
{
  aero::hsi hsi_color;

  // I
  float I_max = std::max({_color.r, _color.g, _color.b});
  hsi_color.i = I_max;
  if (hsi_color.i < target_hsi_min_.i || hsi_color.i > target_hsi_max_.i)
    return false;

  // S
  float i_min = std::min({_color.r, _color.g, _color.b});
  if (I_max > 0) hsi_color.s = 255 * (1 - i_min / I_max);
  else hsi_color.s = 0;
  if (hsi_color.s < target_hsi_min_.s || hsi_color.s > target_hsi_max_.s)
    return false;

  // H
  if (static_cast<int>(I_max) == static_cast<int>(i_min))
  {
    hsi_color.h = 0;
  }
  else
  {
    if (_color.g > _color.b)
    {
      if (_color.r > _color.g)
	hsi_color.h = 60 * (_color.g - _color.b) / (I_max - i_min);
      else
	hsi_color.h = 60 * (2 + (_color.b - _color.r) / (I_max - i_min));
    }
    else
    {
      if (_color.r > _color.b)
	hsi_color.h = 360 - 60 * (_color.b - _color.g) / (I_max - i_min);
      else
	hsi_color.h = 60 * (4 + (_color.r - _color.g) / (I_max - i_min));
    }
    if (hsi_color.h <= 180) // 0 ~ 180 -> 0 ~ 127
      hsi_color.h = static_cast<int>(hsi_color.h / 180.0 * 127);
    else // 180 ~ 360 -> -128 ~ 0
      hsi_color.h = static_cast<int>((hsi_color.h - 360) / 180.0 * 128);
  }
  if (hsi_color.h < target_hsi_min_.h || hsi_color.h > target_hsi_max_.h)
    return false;

  return true;
};
