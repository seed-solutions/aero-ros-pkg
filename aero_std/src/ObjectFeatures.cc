#include "aero_std/ObjectFeatures.hh"

using namespace aero;
using namespace vision;

//////////////////////////////////////////////////
/// @brief constructor
/// @param _nh Node handle
ObjectFeatures::ObjectFeatures(ros::NodeHandle _nh)
  : nh_(_nh)
{
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  camera_relative_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  camera_relative_orientation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

//////////////////////////////////////////////////
/// @brief constructor with AeroMoveitInterface
/// @param _nh Node handle
/// @param _interface AeroMoveitInterface
ObjectFeatures::ObjectFeatures(ros::NodeHandle _nh, aero::interface::AeroMoveitInterface::Ptr _interface)
  : nh_(_nh)
{
  interface_ = _interface;

  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
  camera_relative_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  camera_relative_orientation_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

//////////////////////////////////////////////////
ObjectFeatures::~ObjectFeatures()
{
}

//////////////////////////////////////////////////
/// @brief convert 3d position from in camera coords
///        to in world coords (/base_link)
///        This function requires AeroMoveitInterface
/// @param _pos position in camera coords
/// @param _update_model if true, update the robot model
/// @return position in world coords
Eigen::Vector3d ObjectFeatures::convertWorld(Eigen::Vector3d _pos, bool _update_model)
{

  if(_update_model) {
    interface_->setRobotStateToCurrentState();
    interface_->updateLinkTransforms();
  }
  Eigen::Vector3d p_pos = interface_->kinematic_state->getGlobalLinkTransform(camera_parent_link_).translation();
  Eigen::Matrix3d mat = interface_->kinematic_state->getGlobalLinkTransform(camera_parent_link_).rotation();
  Eigen::Quaterniond p_qua(mat);

  Eigen::Vector3d pos_tmp = camera_relative_orientation_ * _pos;

  return p_pos + p_qua * camera_relative_position_ + p_qua * pos_tmp;
}

//////////////////////////////////////////////////
/// @brief convert 3d position from in camera coords
///        to in world coords (/base_link)
Eigen::Vector3d ObjectFeatures::convertWorld(Eigen::Vector3f _pos, bool _update_model)
{
  return convertWorld(Eigen::Vector3d(_pos.x(), _pos.y(), _pos.z()), _update_model);
}

//////////////////////////////////////////////////
/// @brief set camera transformation,
///        this transform is used in convertWorld()
/// @param _camera_parent_link parent frameID of camera
/// @param _position position of camera
/// @param _orientation orientation of camera
void ObjectFeatures::setCameraTransform(std::string _camera_parent_link, Eigen::Vector3d _position, Eigen::Quaterniond _orientation)
{
  camera_parent_link_ = _camera_parent_link;
  camera_relative_position_ = _position;
  camera_relative_orientation_ = _orientation;
}

//////////////////////////////////////////////////
/// @brief set ARROW to marker,
/// frameID is "/base_link",
/// color is red (1, 0, 0)
///
/// @param _position base position
/// @param _orientation orientation of arrow
/// @param _id marker ID
/// @return marker ID
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
/// @brief set LINE_LIST to marker,
/// frameID is "/base_link",
/// color is blue (0, 0, 1)
///
/// @param _pos1 vertex position (1)
/// @param _pos2 vertex position (2)
/// @param _id marker ID
/// @return marker ID
int ObjectFeatures::setMarker(Eigen::Vector3d _pos1, Eigen::Vector3d _pos2, int _id){
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
/// @brief set LINE_LIST to marker
int ObjectFeatures::setMarker(Eigen::Vector3f _pos1, Eigen::Vector3f _pos2, int _id){
  return setMarker(Eigen::Vector3d(_pos1.x(), _pos1.y(), _pos1.z()), Eigen::Vector3d(_pos2.x(), _pos2.y(), _pos2.z()), _id);
}

//////////////////////////////////////////////////
/// @brief set ARROW to marker, with default rotation (1, 0, 0, 0)
int ObjectFeatures::setMarker(Eigen::Vector3d _position, int _id){
  return setMarker(_position, Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), _id);
}


//////////////////////////////////////////////////
/// @brief set ARROW to marker, with default rotation (1, 0, 0, 0)
int ObjectFeatures::setMarker(Eigen::Vector3f _position, int _id){
  return setMarker(Eigen::Vector3d(_position.x(), _position.y(), _position.z()), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0), _id);
}

//////////////////////////////////////////////////
/// @brief set ARROW to marker
/// @param _pose position and orientation of arrow
int ObjectFeatures::setMarker(geometry_msgs::Pose _pose, int _id){
  return setMarker(Eigen::Vector3d(_pose.position.x, _pose.position.y, _pose.position.z), Eigen::Quaterniond(_pose.orientation.w, _pose.orientation.x, _pose.orientation.y, _pose.orientation.z), _id);
}

//////////////////////////////////////////////////
/// @brief set MESH_RESOURCES to marker
///
/// @param _pose pose in /base_link frame
///
/// @param _mesh_path path to mesh file,
/// File path should be in ros package to display on any PC.
/// ex. _mesh_path = "package://aero_description/meshes/aero_upper_meshes/RARM_LINK7_mesh.dae"
///
/// @param _id marker ID
///
/// @param _color mesh color,
/// If color is (0.0, 0.0, 0.0), mesh file's original color is used.
///
/// @return marker ID
int ObjectFeatures::setMesh(geometry_msgs::Pose _pose, std::string _mesh_path,int _id, std_msgs::ColorRGBA _color){

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "aero_markers";
  marker.id = _id;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = _pose;
  marker.mesh_resource = _mesh_path;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color = _color;
  marker.lifetime = ros::Duration();
  marker.mesh_use_embedded_materials = true;
  marker_publisher_.publish(marker);

  return _id;
}
