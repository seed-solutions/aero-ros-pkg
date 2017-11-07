#include <ros/ros.h>
#include "aero_std/AeroMoveitInterface.hh"

class LookAt {
public: LookAt(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _robot)
  : nh_(_nh) {
  robot_ = _robot;

  set_target_ =
    nh_.subscribe("/look_at/set_target_topic", 1, &LookAt::SetTarget, this);
};

public: ~LookAt() {};

public: void SetTarget(const std_msgs::String::ConstPtr &_msg) {
  sub_.shutdown();
  sub_ = nh_.subscribe(_msg->data, 1, &LookAt::Callback, this);
};

public: void Callback(const geometry_msgs::Point::ConstPtr &_msg) {
  robot_->setLookAt(_msg->x, _msg->y, _msg->z);
  robot_->sendNeckAsync();
  usleep(800 * 1000);
};

private: ros::NodeHandle nh_;

private: ros::Subscriber set_target_;

private: ros::Subscriber sub_;

private: aero::interface::AeroMoveitInterfacePtr robot_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lookat_manager");
  ros::NodeHandle nh;

  aero::interface::AeroMoveitInterfacePtr robot
    (new aero::interface::AeroMoveitInterface(nh));
  std::shared_ptr<LookAt> lookat(new LookAt(nh, robot));

  ros::spin();

  return 0;
}
