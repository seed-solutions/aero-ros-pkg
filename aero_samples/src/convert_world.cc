#include <aero_std/AeroMoveitInterface.hh>
#include <aero_std/ObjectFeatures.hh>

/// @file convert_world.cc
/// @brief How to convert positions from in camera coodinates to in base_link coordinates using robot model.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "minimum_sample_node");
  ros::NodeHandle nh;
  
  // init interfaces
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  aero::vision::ObjectFeaturesPtr features(new aero::vision::ObjectFeatures(nh, robot));
  // wait the publisher connects to rosmaster
  usleep(1000 * 1000);

  // set camera_parent_link and relative camera coordinate.
  // camera_parent_link must exit in the robot model.
  // In this sample, we use a virtual left eye camera. camera is fixed to head.
  features->setCameraTransform("head_base_link", Eigen::Vector3d(0.06, 0.04, 0.08), Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5));

  // make positions on a line in camera coodinates
  std::vector<Eigen::Vector3d> positions_in_camera_coords;
  for(int i=0; i < 10; ++i) {
    Eigen::Vector3d tmp(0.0, 0.0, 0.1 * i + 0.1);
    positions_in_camera_coords.push_back(tmp);
  }

  // convert positions from in camera_link coordinates to in base_link coordinates
  int i =0;
  for(Eigen::Vector3d pos: positions_in_camera_coords) {
    // this method use the robot model to convert coordinates.
    // setRobotStateToCurrentState is called inside, so robot model's state is changed to real robot's state after this method.
    // if you don't wanna change the robot model's state,call `features->convertWorld(pos, false)`. The robot model's state is used instead of real robot state.
    Eigen::Vector3d position_in_base_link_coords = features->convertWorld(pos);

    // Set marker in base_link coordinates.
    features->setMarker(position_in_base_link_coords, i);
    ++i;
  }

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
