#include <aero_std/AeroMoveitInterface.hh>
#include <aero_std/time.h>

/// @file async_cancel_lifter.cc
/// @brief how to send async lifter movement and then cancel

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "trajectory_lifter_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);

  // prepare
  robot->sendLifter(0.0, 0.0);
  usleep(1000 * 1000);

  // send async lifter move
  ROS_INFO("going for -0.4");
  robot->sendLifter(0.0, -0.4, 5000);
  usleep(2500 * 1000);

  // cancel lifter move
  robot->cancelLifter();

  // get current lifter position
  aero::joint_angle_map pos;
  robot->getLifter(pos);
  ROS_INFO("was going for -0.4, cancelled at %f", pos.at(aero::joint::lifter_z));
  usleep(3000 * 1000);

  // end
  robot->sendLifter(0.0, 0.0);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
