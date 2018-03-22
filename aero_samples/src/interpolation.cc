#include <aero_std/AeroMoveitInterface.hh>

/// @file interpolation.cc
/// @brief how to change joint angle's interpolation type.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "interpolation_sample_node");
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


  // change angle's interpolation types
  // all types are listed in aero_std/include/aero_std/interpolation_type.h
  // --- linear ---
  ROS_INFO("type linear");
  robot->setInterpolation(aero::interpolation::i_linear);
  sleep(1);
  robot->sendAngleVector(angles, 5000);
  robot->sendResetManipPose();

  // --- sigmoid ---
  ROS_INFO("type sigmoid");
  robot->setInterpolation(aero::interpolation::i_sigmoid);
  sleep(1);
  robot->sendAngleVector(angles, 5000);
  robot->sendResetManipPose();

  robot->setInterpolation(aero::interpolation::i_constant);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
