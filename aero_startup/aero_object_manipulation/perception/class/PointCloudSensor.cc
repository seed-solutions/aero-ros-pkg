#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"

using namespace aero;
using namespace perception;

//////////////////////////////////////////////////
PointCloudSensor::PointCloudSensor(ros::NodeHandle _nh) : nh_(_nh)
{
  base_to_eye_.position.x = 0;
  base_to_eye_.position.y = 0;
  base_to_eye_.position.z = 0;
  base_to_eye_.orientation.x = 1;
  base_to_eye_.orientation.y = 0;
  base_to_eye_.orientation.z = 0;
  base_to_eye_.orientation.w = 0;

  filter_service_ =
      nh_.advertiseService("/point_cloud/perception_area",
			   &PointCloudSensor::Reconfigure, this);
  points_publisher_ =
      nh_.advertise<std_msgs::Float32MultiArray>("/point_cloud/points", 1000);
  camera_pseudo_tf_subscriber_ =
      nh_.subscribe("/matrix/base_to_eye", 100,
		    &PointCloudSensor::SubscribeCameraPseudoTf, this);
  timing_ = false;
}

//////////////////////////////////////////////////
PointCloudSensor::~PointCloudSensor()
{
}

//////////////////////////////////////////////////
void PointCloudSensor::SubscribeCameraPseudoTf(
    const geometry_msgs::Pose::ConstPtr& _pose)
{
  base_to_eye_.position.x = _pose->position.x;
  base_to_eye_.position.y = _pose->position.y;
  base_to_eye_.position.z = _pose->position.z;
  base_to_eye_.orientation.x = _pose->orientation.x;
  base_to_eye_.orientation.y = _pose->orientation.y;
  base_to_eye_.orientation.z = _pose->orientation.z;
  base_to_eye_.orientation.w = _pose->orientation.w;
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
  space_max_ = {_value.x, _value.y, _value.z};
}

//////////////////////////////////////////////////
void PointCloudSensor::SetSpaceMin(aero::xyz _value)
{
  space_min_ = {_value.x, _value.y, _value.z};
}

//////////////////////////////////////////////////
void PointCloudSensor::SetHSIMax(aero::hsi _value)
{
  target_hsi_max_ = {_value.h, _value.s, _value.i};
}

//////////////////////////////////////////////////
void PointCloudSensor::SetHSIMin(aero::hsi _value)
{
  target_hsi_min_ = {_value.h, _value.s, _value.i};
}
