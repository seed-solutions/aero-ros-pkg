#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"

using namespace aero;
using namespace perception;

//////////////////////////////////////////////////
PointCloudSensor::PointCloudSensor(ros::NodeHandle _nh)
    : aero::common::Base(_nh)
{
  filter_service_ =
      nh_.advertiseService("/point_cloud/perception_area",
			   &PointCloudSensor::Reconfigure, this);
  points_publisher_ =
      nh_.advertise<std_msgs::Float32MultiArray>("/point_cloud/points", 1000);
  timing_ = false;
}

//////////////////////////////////////////////////
PointCloudSensor::~PointCloudSensor()
{
}

//////////////////////////////////////////////////
bool PointCloudSensor::Reconfigure(aero_startup::PointXYZHSI::Request  &_req,
				   aero_startup::PointXYZHSI::Response &_res)
{
  target_hsi_max_.h = _req.h_cap;
  target_hsi_min_.h = _req.h;
  target_hsi_max_.s = _req.s_cap;
  target_hsi_min_.s = _req.s;
  target_hsi_max_.i = _req.i_cap;
  target_hsi_min_.i = _req.i;
  space_max_.x = _req.x_cap;
  space_min_.x = _req.x;
  space_max_.y = _req.y_cap;
  space_min_.y = _req.y;
  space_max_.z = _req.z_cap;
  space_min_.z = _req.z;
  _res.status = 1;
  return true;
}

//////////////////////////////////////////////////
bool PointCloudSensor::ProcessSleep(aero_startup::ProcessSleep::Request  &_req,
				    aero_startup::ProcessSleep::Response &_res)
{
  ROS_WARN("in sensor : %d", _req.sleep);

  sleep_ = _req.sleep;
  if (sleep_) ROS_WARN("sleeping sensor process");
  else ROS_WARN("activating sensor process");
  _res.status = aero::status::success;
  return true;
}

//////////////////////////////////////////////////
bool PointCloudSensor::GetTiming()
{
  return timing_;
}

//////////////////////////////////////////////////
Eigen::Vector3f PointCloudSensor::GetCenter()
{
  return center_;
}

//////////////////////////////////////////////////
std::vector<Eigen::Vector3f> PointCloudSensor::GetVertices()
{
  return vertices_;
}

//////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudSensor::GetCloud()
{
  return cloud_;
}

//////////////////////////////////////////////////
std::vector<aero::rgb> PointCloudSensor::GetRGB()
{
  return rgb_;
}

//////////////////////////////////////////////////
void PointCloudSensor::SetSpaceMax(aero::xyz _value)
{
  space_max_ = _value;
}

//////////////////////////////////////////////////
void PointCloudSensor::SetSpaceMin(aero::xyz _value)
{
  space_min_ = _value;
}

//////////////////////////////////////////////////
void PointCloudSensor::SetHSIMax(aero::hsi _value)
{
  target_hsi_max_ = _value;
}

//////////////////////////////////////////////////
void PointCloudSensor::SetHSIMin(aero::hsi _value)
{
  target_hsi_min_ = _value;
}
