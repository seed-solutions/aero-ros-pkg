#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

bool in_action_ = false;

void callback(const std_msgs::Bool::ConstPtr& _msg) {
  in_action_ = _msg->data;
}

bool service(std_srvs::Trigger::Request &_req,
             std_srvs::Trigger::Response &_res) {
  _res.success = in_action_;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wait_interpolation_manager");
  ros::NodeHandle nh;

  ros::Subscriber sub =
    nh.subscribe("/aero_controller/in_action", 1, &callback);

  ros::ServiceServer s =
    nh.advertiseService("/aero_controller/get_in_action", service);

  ros::Rate r(20);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
