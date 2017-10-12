/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi

#include <ros/ros.h>
#include "aero_move_base/AeroMoveBase.hh"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheel_controller");
  ros::NodeHandle nh;

  aero::navigation::AeroMoveBasePtr aero(new aero::navigation::AeroMoveBase(nh));

  ros::spin();

  return 0;
}
