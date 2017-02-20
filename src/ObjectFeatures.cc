#include "ObjectFeatures.hh"

using namespace aero;
using namespace vision;

//////////////////////////////////////////////////
ObjectFeatures::ObjectFeatures(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr interface_ptr)
  : nh_(_nh)
{
  interface_ = interface_ptr;
}

//////////////////////////////////////////////////
ObjectFeatures::~ObjectFeatures()
{
}

//////////////////////////////////////////////////
Eigen::Vector3f ObjectFeatures::ConvertWorld(Eigen::Vector3f _pos_camera)
{
  Eigen::Quaternionf base_to_eye_q =
    Eigen::Quaternionf(base_to_eye_.orientation.x,
		       base_to_eye_.orientation.y,
		       base_to_eye_.orientation.z,
		       base_to_eye_.orientation.w);

  return
    Eigen::Vector3f(base_to_eye_.position.x,
		    base_to_eye_.position.y,
		    base_to_eye_.position.z) + base_to_eye_q * _pos_camera;
}
