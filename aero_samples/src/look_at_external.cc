#include <aero_std/AeroMoveitInterface.hh>

/// @file look_at_external.cc
/// @brief how to control neck to a positioned target while moving body
/// external node required: rosrun aero_std look_at.

int main(int argc, char **argv)
{
  // setups for example

  // init ros
  ros::init(argc, argv, "look_at_external_sample_node");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterfacePtr robot(new aero::interface::AeroMoveitInterfaceDeprecated(nh));

  // reset robot pose
  ROS_INFO("reseting robot pose");

  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  // create target to look at
  aero::Transform obj0;
  robot->getEndEffectorCoords(aero::arm::rarm, aero::eef::pick, obj0);

  // main example starts here

  // set positioned look at on background
  ROS_INFO("set tracking mode -> true");
  robot->setTrackingMode(true);
  // set position to track: map_coordinate=false, tracking=true
  robot->setLookAt(obj0.translation(), false, true);


  // change robot pose (lookAt should be updated in background)
  ROS_INFO("moving torso"); // send to different pose
  aero::joint_angle_map joints2;
  robot->setJoint(aero::joint::waist_p, 0.524);
  robot->setRobotStateVariables(joints2);
  robot->sendModelAngles(5000);
  sleep(2);

  // set background tracking to false
  ROS_INFO("set tracking mode -> false");
  robot->setTrackingMode(false);
  sleep(2); // wait for background thread to completely finish


  ROS_INFO("reseting robot pose");
  //robot->sendResetManipPose();
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  ROS_INFO("set tracking mode");
  robot->setTrackingMode(true);
  robot->setLookAt(aero::Vector3(1.5, 0, 0), false, true); // _map = false, _tarcking = true

  ROS_INFO("wait");
  robot->waitInterpolation();

  ROS_INFO("send 1");
  robot->setJoint(aero::joint::waist_y, 1.0);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  ROS_INFO("send 2");
  robot->setJoint(aero::joint::waist_y, -1.0);
  robot->sendModelAngles(6000);
  robot->waitInterpolation();

  robot->setTrackingMode(false);

  ROS_INFO("reseting robot pose");
  //robot->sendResetManipPose();
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(2000);
  robot->waitInterpolation();
  ROS_INFO("demo node finished");

  ros::shutdown();
  return 0;
}
