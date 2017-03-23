#include <aero_std/AeroMoveitInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "sendAV_async_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr interface(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  interface->sendResetManipPose();
  sleep(1);

  double l_elbow_to = -1.745;
  std::map<aero::joint, double> joint_angles;
  interface->getRobotStateVariables(joint_angles);// save angles from robot model
  joint_angles[aero::joint::l_elbow] = l_elbow_to;// replace elbow's angle value

  interface->sendAngleVectorAsync(joint_angles, 5000);
  ROS_INFO("this motion takes 5 seconds");
  for (int i = 0; i < 5; ++i) {
    ROS_INFO("%d", 5-i);
    usleep(1000 * 1000);
  }
  usleep(1000 * 1000);
  interface->waitSendAngleVectorAsync();
  ROS_INFO("finished");

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
