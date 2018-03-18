#include <aero_std/AeroMoveitInterface.hh>

/// @file hand.cc
/// @brief how to controll hand. move to desired angle, grasp, and open

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "hand_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);


  // hand functions
  // move hands to selected angle[rad]
  // sendHand(arm, radian)
  ROS_INFO("moving hands");
  double to_rad = M_PI / 180.0;
  ROS_INFO("move to %d degrees", 0);
  robot->sendHand(aero::arm::rarm, 0 * to_rad);
  robot->sendHand(aero::arm::larm, 0 * to_rad);
  sleep(3);
  ROS_INFO("move to %d degrees", 30);
  robot->sendHand(aero::arm::rarm, 30.0 * to_rad);
  robot->sendHand(aero::arm::larm, 30.0 * to_rad);
  sleep(3);
  ROS_INFO("move to %d degrees", -30);
  robot->sendHand(aero::arm::rarm, -30.0 * to_rad);
  robot->sendHand(aero::arm::larm, -30.0 * to_rad);
  sleep(3);


  // grasping
  ROS_INFO("left hand grasping 5 seconds after");
  ROS_INFO("if no object exists in the hand, warn message will appear");
  sleep(5);
  robot->sendGrasp(aero::arm::larm, 50);// arm and grasping power(max 100[%])
  sleep(5);

  // ungrasping
  ROS_INFO("left hand ungrasping 5 seconds after");
  ROS_INFO("please be careful not to drop objects");
  sleep(5);
  robot->openHand(aero::arm::larm);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
