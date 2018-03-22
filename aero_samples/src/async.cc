#include <aero_std/AeroMoveitInterface.hh>

/// @file async.cc
/// @brief send joint angles in async mode.

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

  // preparation
  aero::joint_angle_map angles;
  robot->getRobotStateVariables(angles);
  angles[aero::joint::r_shoulder_p] = -1.4;
  angles[aero::joint::r_shoulder_r] = -0.7854;
  angles[aero::joint::r_elbow]= -0.2618;

  // send first angles with 5 seconds
  robot->sendAngleVectorAsync(angles, 5000);
  sleep(3);

  // overwrite angles before reaching goal
  robot->sendResetManipPose();

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
