#include "aero_teleop/xbox_one_teleop.hh"

int main(int argc, char *argv[])
{
  ROS_INFO("initializing robot ...");

  ros::init(argc, argv, "teleop_joy");
  ros::NodeHandle nh_param;
  aero::teleop::xbox_one_teleop::Ptr joy
    (new aero::teleop::xbox_one_teleop(nh_param));

  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    joy->loop();
    r.sleep();
  }
}
