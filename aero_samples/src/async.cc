#include <aero_std/AeroMoveitInterface.hh>

/// @file async.cc
/// @brief send joint angles in async mode.

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


  // preparation
  aero::joint_angle_map angles;
  robot->getRobotStateVariables(angles);
  angles[aero::joint::r_shoulder_p] = -1.4;
  angles[aero::joint::r_shoulder_r] = -0.7854;
  angles[aero::joint::r_elbow]= -0.2618;

  // send first angles with 5 seconds
  robot->setRobotStateVariables(angles);
  robot->sendAngleVector(5000);
  sleep(3);

  // overwrite angles before reaching goal
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
