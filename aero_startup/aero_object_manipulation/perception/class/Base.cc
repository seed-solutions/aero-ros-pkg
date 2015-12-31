#include "Base.hh"

using namespace aero;
using namespace common;

//////////////////////////////////////////////////
Base::Base(ros::NodeHandle _nh) : nh_(_nh)
{
  base_to_eye_.position.x = 0;
  base_to_eye_.position.y = 0;
  base_to_eye_.position.z = 0;
  base_to_eye_.orientation.x = 1;
  base_to_eye_.orientation.y = 0;
  base_to_eye_.orientation.z = 0;
  base_to_eye_.orientation.w = 0;

  camera_pseudo_tf_subscriber_ =
    nh_.subscribe("/matrix/base_to_eye", 100,
		  &Base::SubscribeCameraPseudoTf, this);
}

//////////////////////////////////////////////////
Base::~Base()
{
}

//////////////////////////////////////////////////
void Base::SubscribeCameraPseudoTf(
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
