#include <hrpsys_aero_bridge/aero_controller_node.hpp>

namespace aero_controller {

AeroControllerNode::AeroControllerNode(
    ros::NodeHandle& nh, ros::NodeHandle& pnh,
    std::string port_upper, std::string port_lower) :
    controller_(port_upper, port_lower),
    nh_(nh), pnh_(pnh)
{
  cmdvel_sub_ =
      pnh.subscribe<geometry_msgs::Twist>(
          "cmd_vel", 100, &AeroControllerNode::goVelocityCallback, this);
  jointtraj_sub_ =
      pnh.subscribe<trajectory_msgs::JointTrajectory>(
          "command", 100, &AeroControllerNode::jointTrajectoryCallback, this);
}

AeroControllerNode::~AeroControllerNode() {
}

void AeroControllerNode::goVelocityCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {

}
void AeroControllerNode::jointTrajectoryCallback(
    const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
  for (size_t i = 0; i < msg->joint_names.size(); i++) {
    std::string joint_name = msg->joint_names[i];
  }

  for (size_t i = 0; i < msg->points; i++) {

  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "aero_controller");

  ros::NodeHandle n;
  ros::NodeHandle np("~");

  aero_controller::AeroControllerNode(
      n, np, std::string("/dev/ttyUSB0"), std::string("/dev/ttyUSB1"));

  ros::spin();

  return 0;
}
