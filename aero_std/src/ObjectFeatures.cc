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
Eigen::Vector3d ObjectFeatures::convertWorld(Eigen::Vector3d _pos)
{

  interface_->setRobotStateToCurrentState();
  interface_->updateLinkTransforms();
  Eigen::Vector3d p_pos = interface_->kinematic_state->getGlobalLinkTransform(camera_parent_link_).translation();
  Eigen::Matrix3d mat = interface_->kinematic_state->getGlobalLinkTransform(camera_parent_link_).rotation();
  Eigen::Quaterniond p_qua(mat);

  Eigen::Vector3d pos_tmp = camera_relative_orientation_ * _pos;

  return p_pos + p_qua * camera_relative_position_ + p_qua * pos_tmp;
}

//////////////////////////////////////////////////
Eigen::Vector3d ObjectFeatures::convertWorld(Eigen::Vector3f _pos)
{
  return convertWorld(Eigen::Vector3d(_pos.x(), _pos.y(), _pos.z()));
}

//////////////////////////////////////////////////
void ObjectFeatures::setCameraTransform(std::string _camera_parent_link, Eigen::Vector3d _position, Eigen::Quaterniond _orientation)
{
  camera_parent_link_ = _camera_parent_link;
  camera_relative_position_ = _position;
  camera_relative_orientation_ = _orientation;
}

//////////////////////////////////////////////////
int ObjectFeatures::setMarker(Eigen::Vector3d _position, Eigen::Quaterniond _orientation, int _id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "aero_markers";
  marker.id = _id;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = _position.x();
  marker.pose.position.y = _position.y();
  marker.pose.position.z = _position.z();
  marker.pose.orientation.x = _orientation.x();
  marker.pose.orientation.y = _orientation.y();
  marker.pose.orientation.z = _orientation.z();
  marker.pose.orientation.w = _orientation.w();
  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  marker_publisher_.publish(marker);

  return _id;
}

//////////////////////////////////////////////////
int ObjectFeatures::setMarker(Eigen::Vector3f _pos1, Eigen::Vector3f _pos2, int _id){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "aero_markers";
  marker.id = _id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.02;
  geometry_msgs::Point p1;
  p1.x = _pos1.x(); p1.y = _pos1.y(); p1.z = _pos1.z();
  geometry_msgs::Point p2;
  p2.x = _pos2.x(); p2.y = _pos2.y(); p2.z = _pos2.z();
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker_publisher_.publish(marker);

  return _id;
}

//////////////////////////////////////////////////
int ObjectFeatures::setMarker(Eigen::Vector3d _position, int _id){
  return setMarker(_position, Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), _id);
}


//////////////////////////////////////////////////
int ObjectFeatures::setMarker(Eigen::Vector3f _position, int _id){
  return setMarker(Eigen::Vector3d(_position.x(), _position.y(), _position.z()), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), _id);
}

//////////////////////////////////////////////////
int ObjectFeatures::setMarker(geometry_msgs::Pose _pose, int _id){
  return setMarker(Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z), Eigen::Quaterniond(_pose.orientation.w, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z), _id);
}
