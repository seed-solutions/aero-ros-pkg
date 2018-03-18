#include <aero_std/AeroMoveitInterface.hh>

/// @file minimum.cc
/// @brief minimum code to use interface. This file initialize interface and move robot once.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "minimum_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
