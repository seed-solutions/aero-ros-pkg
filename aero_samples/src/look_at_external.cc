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
  aero::interface::AeroMoveitInterfacePtr robot(new aero::interface::AeroMoveitInterface(nh));

  // reset robot pose
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);
  // create target to look at
  Eigen::Vector3d obj0 = robot->getEEFPosition(aero::arm::rarm, aero::eef::pick);



  // main example starts here

  // set positioned look at on background
  robot->setTrackingMode(true);
  // set position to track: map_coordinate=false, tracking=true
  robot->setLookAt(obj0, false, true);

  // change robot pose (lookAt should be updated in background)
  ROS_INFO("moving torso"); // send to different pose
  aero::joint_angle_map joints2;
  joints2[aero::joint::waist_p] = 0.524;
  robot->setRobotStateVariables(joints2);
  robot->sendAngleVector(5000);
  sleep(1);

  // set background tracking to false
  robot->setTrackingMode(false);
  sleep(2); // wait for background thread to completely finish

  // finish
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
