#include <aero_std/AeroMoveitInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "set_neck_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr interface(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  interface->sendResetManipPose();
  sleep(1);

  // how to use setNeck(roll pitch yaw);

  ROS_INFO("looking down");
  interface->setNeck(0.0, 0.3, 0.0);// set robot model's neck joints' value
  interface->sendAngleVector(2000);// finally, send robot model's joints values to real robot. neck angles are sended with body angles
  sleep(3);

  ROS_INFO("looking left");
  interface->setNeck(0.0, 0.0, 100.0);// too much value is redued to joint limit
  interface->sendAngleVector(2000);
  sleep(3);

  // reset neck angles
  interface->resetLookAt();// this code calls setNeck(0.0, 0.0, 0.0)
  interface->sendAngleVector(2000);// neck angles sended to real robot with body angles
  //interface->sendResetManipPose(); when sendResetManipPose is called, neck angles are alse sended to real robot

  ROS_INFO("reseting pose");
  interface->sendResetManipPose();
  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
