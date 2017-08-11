#include <aero_std/AeroMoveitInterface.hh>

/// @file look_at.cc
/// @brief how to controll neck and look at object

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "look_at_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr interface(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  interface->sendResetManipPose();
  sleep(1);

  // preparation
  std::map<aero::joint, double> joints_rh, joints_lh;
  interface->getRobotStateVariables(joints_rh);
  joints_lh = joints_rh;

  joints_rh[aero::joint::r_elbow] = -1.745;
  joints_lh[aero::joint::l_elbow] = -1.745;



  // how to use setLookAt
  //
  // in this code, aero looks at his hand

  // looks at right hand
  ROS_INFO("look at right hand");
  interface->setRobotStateVariables(joints_rh);// first, set robot model's joints except head's joints
  Eigen::Vector3d obj_rh = interface->getEEFPosition(aero::arm::rarm, aero::eef::pick);// second, prepare target position
  interface->setLookAt(obj_rh);// third, set lookAt target to robot model
  interface->sendAngleVector(1000);// finally, send robot model's joints values to real robot. neck angles are sended with body angles
  sleep(3);

  // looks at left hand
  ROS_INFO("look at left hand");
  interface->setRobotStateVariables(joints_lh);
  interface->setLookAt(interface->getEEFPosition(aero::arm::larm, aero::eef::pick));
  interface->sendAngleVector(1000);
  sleep(3);

  // reset neck angles
  interface->resetLookAt();// this time, neck doesn't move
  interface->sendAngleVector(1000);// neck angles sended to real robot with body angles
  //interface->sendResetManipPose(); when sendResetManipPose is called, neck angles are alse sended to real robot

  ROS_INFO("reseting pose");
  interface->sendResetManipPose();
  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
