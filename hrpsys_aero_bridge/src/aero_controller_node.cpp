#include <hrpsys_aero_bridge/aero_controller_node.hpp>

namespace aero_controller {

AeroControllerNode::AeroControllerNode(
    const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
    std::string port_upper, std::string port_lower) :
    controller_(port_upper, port_lower),
    nh_(nh), pnh_(pnh)
{
  ROS_INFO("starting aero_controller");
  cmdvel_sub_ =
      pnh_.subscribe(
          "cmd_vel", 100, &AeroControllerNode::goVelocityCallback, this);
  jointtraj_sub_ =
      pnh_.subscribe(
          "command", 100, &AeroControllerNode::jointTrajectoryCallback, this);
}

AeroControllerNode::~AeroControllerNode() {
}

void AeroControllerNode::goVelocityCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {

}
void AeroControllerNode::jointTrajectoryCallback(
    const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
  std::vector<size_t> joint_to_stroke_indices;
  for (size_t i = 0; i < msg->joint_names.size(); i++) {
    std::string joint_name = msg->joint_names[i];
    int32_t stroke_idx =
        controller_.get_stroke_index_from_joint_name(joint_name);
    if (stroke_idx >= 0) {
      joint_to_stroke_indices.push_back(static_cast<size_t>(stroke_idx));
    }
  }

  // if joint stroke is undefined, use previous stroke
  std::vector<int16_t> stroke_vector;
  std::vector<int16_t>& ref_vector = controller_.get_reference_stroke_vector();
  stroke_vector.assign(ref_vector.begin(), ref_vector.end());
  for (size_t i = 0; i < msg->points.size(); i++) {
    for (size_t j = 0; j < msg->points[i].positions.size(); j++) {
      stroke_vector[joint_to_stroke_indices[j]] =
          static_cast<int16_t>(100.0 * msg->points[i].positions[j]);
    }
    double time_sec = msg->points[i].time_from_start.toSec();
    uint16_t time_msec = static_cast<uint16_t>(time_sec * 1000.0);
    controller_.set_position(stroke_vector, time_msec);
    usleep(static_cast<int32_t>(time_sec * 1000.0 * 1000.0));
  }
}

}  // namespace


int main(int argc, char** argv) {
  ros::init(argc, argv, "aero_controller");

  ros::NodeHandle n;
  ros::NodeHandle np("~");

  aero_controller::AeroControllerNode(
      //n, np, std::string("/dev/ttyUSB0"), std::string("/dev/ttyUSB1"));
      n, np, std::string(""), std::string(""));

  ros::spin();

  return 0;
}
