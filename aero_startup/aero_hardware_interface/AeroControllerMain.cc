# include <ros/ros.h>
# include "aero_controller/AeroControllerNode.hh"

int main(int argc, char** argv) {
  ros::init(argc, argv, "aero_controller");
  ros::NodeHandle nh("~");

  std::string port_upper("/dev/aero_upper");
  std::string port_lower("/dev/aero_lower");

  bool debug = false;

  nh.param<bool> ("debug", debug, false);

  if (debug)
  {
    port_upper = "";
    port_lower = "";
  }
  else
  {
    nh.param<std::string> ("port_upper", port_upper, "/dev/aero_upper");
    nh.param<std::string> ("port_lower", port_lower, "/dev/aero_lower");
  }

  ROS_INFO_STREAM("initializing ctrl with " <<
                  port_upper << " and " << port_lower);

  aero::controller::AeroControllerNodePtr aero(
      new aero::controller::AeroControllerNode(nh, port_upper, port_lower));

  ROS_INFO("configuring done");

  ros::spin();

  return 0;
}
