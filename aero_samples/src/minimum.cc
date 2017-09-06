#include <aero_std/AeroMoveitInterface.hh>

/// @file minimum.cc
/// @brief minimum code to use interface. This file initialize interface and move robot once.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "minimum_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
