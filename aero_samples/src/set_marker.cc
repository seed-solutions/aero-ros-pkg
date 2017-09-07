#include <aero_std/ObjectFeatures.hh>

/// @file set_marker.cc
/// @brief sample code to set marker. You can see markers on rviz.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "set_marker_sample_node");
  ros::NodeHandle nh;
  
  // init object marker publisher
  aero::vision::ObjectFeaturesPtr features(new aero::vision::ObjectFeatures(nh));

  sleep(1);

  //----------
  // ex1 pose
  // make marker on rviz
  geometry_msgs::Pose pose;
  pose.position.x = 2.0;
  pose.position.z = 1.0;

  // publish marker with pose and id
  features->setMarker(pose, 1);

  //----------
  // ex2 position
  // make marker on rviz
  Eigen::Vector3d pos;
  pos.x() = 3.0;
  pos.z() = 1.0;
  features->setMarker(pos, 2);

  //----------
  // ex3 line
  // draw line on rviz
  Eigen::Vector3d from,to;
  from.x() = 4.0;
  from.z() = 0.0;
  to.x() = 4.0;
  to.z() = 2.0;
  features->setMarker(from, to, 3);

  //----------
  // ex4 mesh
  // spawn mesh on rviz
  geometry_msgs::Pose pose4;
  pose4.position.x = 5.0;
  pose4.position.z = 1.0;
  features->setMesh(pose4, "package://aero_samples/models/ring.dae", 4);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
