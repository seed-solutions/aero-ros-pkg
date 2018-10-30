#include <aero_std/AeroMoveitInterface.hh>

/// @file look_at.cc
/// @brief how to controll neck and look at object

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "look_at_sample_node");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  //
  ROS_INFO("setNeck");
  robot->setNeck(0, 0.3, 0.5);
  robot->sendModelAngles(2000);
  robot->waitInterpolation();

  // preparation
  aero::joint_angle_map joints_rh, joints_lh;
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->setJoint(aero::joint::r_elbow, -1.745);
  robot->getRobotStateVariables(joints_rh);

  robot->setPoseVariables(aero::pose::reset_manip);
  robot->setJoint(aero::joint::l_elbow, -1.745);
  robot->getRobotStateVariables(joints_lh);

  // how to use setLookAt
  // in this code, aero looks at his hand

  // looks at right hand
  ROS_INFO("look at right hand");
  robot->setRobotStateVariables(joints_rh);// first, set robot model's joints except head's joints
  aero::Vector3 obj_rh = robot->getEEFPosition(aero::arm::rarm, aero::eef::pick);// second, prepare target position
  robot->setLookAt(obj_rh);// third, set lookAt target to robot model
  robot->sendModelAngles(2000);// finally, send robot model's joints values to real robot. neck angles are sended with body angles
  robot->waitInterpolation();

  // looks at left hand
  ROS_INFO("look at left hand");
  robot->setRobotStateVariables(joints_lh);
  robot->setLookAt(robot->getEEFPosition(aero::arm::larm, aero::eef::pick));
  robot->sendModelAngles(2000);
  robot->waitInterpolation();

  // reset neck angles
  robot->resetLookAt();// this time, neck doesn't move
  robot->sendModelAngles(1000);// neck angles sended to real robot with body angles
  robot->waitInterpolation();
  //robot->sendResetManipPose(); when sendResetManipPose is called, neck angles are alse sended to real robot

  ROS_INFO("reseting pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  ROS_INFO("demo node finished");
  ros::shutdown();

  return 0;
}
