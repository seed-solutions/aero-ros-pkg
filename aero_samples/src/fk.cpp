#include <aero_std/AeroMoveitInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "fk_saple_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr interface(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  interface->sendResetManipPose();
  sleep(1);

  // set real robot's joint angles to the robot model in interface 
  interface->setRobotStateToCurrentState();

  // how to move selected joint
  double l_elbow_to = -1.745;
  std::map<aero::joint, double> joint_angles;
  interface->getRobotStateVariables(joint_angles);// save angles from robot model
  joint_angles[aero::joint::l_elbow] = l_elbow_to;// replace elbow's angle value
  ROS_INFO("left elbow moves from %d to %d", joint_angles[aero::joint::l_elbow], l_elbow_to);


  interface->sendAngleVectorAsync(joint_angles, 2000);// send to robot
  ROS_INFO("moveing left elbow");
  sleep(5);


  ROS_INFO("reseting robot pose");
  interface->sendResetManipPose();
  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
