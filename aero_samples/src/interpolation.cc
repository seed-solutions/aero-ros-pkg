#include <aero_std/AeroMoveitInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "interpolation_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr interface(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  interface->sendResetManipPose();
  sleep(1);

  // preparation
  std::map<aero::joint, double> angles;
  interface->getRobotStateVariables(angles);
  angles[aero::joint::r_shoulder_p] = -1.4;
  angles[aero::joint::r_shoulder_r] = -0.7854;
  angles[aero::joint::r_elbow]= -0.2618;


  // change angle's interpolation types
  // all types are listed in aero_std/include/aero_std/interpolation_type.h
  // --- linear ---
  ROS_INFO("type linear");
  interface->setInterpolation(aero::interpolation::i_linear);
  sleep(1);
  interface->sendAngleVector(angles, 5000);
  interface->sendResetManipPose();

  // --- bezier ---
  ROS_INFO("type sigmoid");
  interface->setInterpolation(aero::interpolation::i_sigmoid);
  sleep(1);
  interface->sendAngleVector(angles, 5000);
  interface->sendResetManipPose();

  interface->setInterpolation(aero::interpolation::i_constant);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
