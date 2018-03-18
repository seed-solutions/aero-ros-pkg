#include <aero_std/AeroMoveitInterface.hh>

/// @file interpolation.cc
/// @brief how to change joint angle's interpolation type.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "interpolation_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  //robot->sendResetManipPose();
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);

  // preparation
  aero::joint_angle_map angles;
  robot->getRobotStateVariables(angles);
  angles[aero::joint::r_shoulder_p] = -1.4;
  angles[aero::joint::r_shoulder_r] = -0.7854;
  angles[aero::joint::r_elbow]= -0.2618;


  // change angle's interpolation types
  // all types are listed in aero_std/include/aero_std/interpolation_type.h
  // --- linear ---
  ROS_INFO("type linear");
  //robot->setInterpolation(aero::interpolation::i_linear); // TODO: not implemented yet
  sleep(1);
  robot->setRobotStateVariables(angles);
  robot->sendAngleVector(5000);
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);

  // --- sigmoid ---
  ROS_INFO("type sigmoid");
  //robot->setInterpolation(aero::interpolation::i_sigmoid); // TODO: not implemented yet
  sleep(1);
  robot->setRobotStateVariables(angles);
  robot->sendAngleVector(5000);
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);

  //robot->setInterpolation(aero::interpolation::i_constant); // TODO: not implemented yet

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
