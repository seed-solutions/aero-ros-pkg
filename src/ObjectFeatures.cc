#include "aero_std/ObjectFeatures.hh"

using namespace aero;
using namespace vision;

//////////////////////////////////////////////////
ObjectFeatures::ObjectFeatures(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _interface)
  : nh_(_nh)
{
  interface_ = _interface;

  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  camera_relative_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  camera_relative_orientation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

//////////////////////////////////////////////////
ObjectFeatures::~ObjectFeatures()
{
}

//////////////////////////////////////////////////
Eigen::Vector3d ObjectFeatures::ConvertWorld(Eigen::Vector3d _pos)
{

  interface_->setRobotStateToCurrentState();

  Eigen::Vector3d p_pos = interface_->kinematic_state->getGlobalLinkTransform(camera_parent_link_).translation();

  Eigen::Matrix3d mat = interface_->kinematic_state->getGlobalLinkTransform(camera_parent_link_).rotation();
  Eigen::Quaterniond p_qua(mat);

  Eigen::Vector3d pos_tmp = camera_relative_orientation_ * _pos;

  return p_pos + p_qua * camera_relative_position_ + p_qua * pos_tmp;
    //    p_pos + p_qua * camera_relative_position_ + p_qua * camera_relative_position_ * _pos;
}

//////////////////////////////////////////////////
void ObjectFeatures::setCameraTransform(std::string _camera_parent_link, Eigen::Vector3d _position, Eigen::Quaterniond _orientation)
{
  camera_parent_link_ = _camera_parent_link;
  camera_relative_position_ = _position;
  camera_relative_orientation_ = _orientation;
}

//////////////////////////////////////////////////
int ObjectFeatures::setMarker(geometry_msgs::Point _position, int _id){
  return _id;
}
