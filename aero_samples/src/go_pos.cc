#include <aero_std/AeroMoveitInterface.hh>

/// @file go_pos.cc
/// @brief sample node of go_pos. 
/// @attention please launch aero_startup/wheel_with_making_map.launch, when executing this node.

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

  // go 0.5 meters forward
  robot->goPos(0.5, 0.0, 0.0);

  usleep(2000 * 1000);

  robot->goPos(-0.5, 0.0, 0.0);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
