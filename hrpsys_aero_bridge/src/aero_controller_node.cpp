#include <hrpsys_aero_bridge/aero_controller_node.hpp>

namespace aero_controller {

AeroControllerNode::AeroControllerNode(const ros::NodeHandle& nh,
                                       const std::string& port_upper,
                                       const std::string& port_lower) :
    handle_(nh), controller_(port_upper, port_lower)
{
  ROS_INFO("starting aero_controller");
  ROS_INFO(" create publisher");
  state_pub_ =
      handle_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(
          "state", 10);

  ROS_INFO(" create cmdvel sub");
  cmdvel_sub_ =
      handle_.subscribe(
          "cmd_vel",
          10,
          &AeroControllerNode::GoVelocityCallback,
          this);
  ROS_INFO(" create command sub");
  jointtraj_sub_ =
      handle_.subscribe(
          "command",
          10,
          &AeroControllerNode::JointTrajectoryCallback,
          this);

  ROS_INFO(" create timer sub");
  timer_ =
      handle_.createTimer(ros::Duration(0.1),
                     &AeroControllerNode::JointStateCallback, this);
  ROS_INFO(" done");
}

AeroControllerNode::~AeroControllerNode() {
}

void AeroControllerNode::GoVelocityCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  {
    boost::mutex::scoped_lock lock(mtx_);
    controller_.flush();
  }
  usleep(1000 * 10);
}

void AeroControllerNode::JointTrajectoryCallback(
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
    {
      boost::mutex::scoped_lock lock(mtx_);
      controller_.flush();
      controller_.set_position(stroke_vector, time_msec);
    }
    usleep(static_cast<int32_t>(time_sec * 1000.0 * 1000.0));
  }
}

void AeroControllerNode::JointStateCallback(const ros::TimerEvent& event) {
  pr2_controllers_msgs::JointTrajectoryControllerState state;
  state.header.stamp = ros::Time::now();
  state.joint_names.resize(AERO_DOF);
  state.desired.positions.resize(AERO_DOF);
  state.actual.positions.resize(AERO_DOF);

  std::vector<int16_t> stroke_vector;
  std::vector<int16_t>& ref_vector =
      controller_.get_reference_stroke_vector();
  {
    boost::mutex::scoped_lock lock(mtx_);
    controller_.flush();
    controller_.get_position(stroke_vector);
  }
  for (size_t i = 0; i < AERO_DOF; i++) {
    state.joint_names[i] = controller_.get_joint_name(i);
    state.desired.positions[i] = static_cast<double>(ref_vector[i]) * 0.01;
    state.actual.positions[i] = static_cast<double>(stroke_vector[i]) * 0.01;
  }
  state_pub_.publish(state);
}

}  // namespace


int main(int argc, char** argv) {
  ros::init(argc, argv, "aero_controller");
  ros::NodeHandle np("~");

  std::string port_upper("/dev/ttyUSB0");
  std::string port_lower("/dev/ttyUSB1");
  aero_controller::AeroControllerNode aero(np, port_upper, port_lower);
  ROS_INFO("configuring done");

  ros::spin();

  return 0;
}
