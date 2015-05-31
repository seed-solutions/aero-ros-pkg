#include <hrpsys_aero_bridge/aero_controller_node.hpp>

namespace aero_controller {

AeroControllerNode::AeroControllerNode(const ros::NodeHandle& nh,
                                       const std::string& port_upper,
                                       const std::string& port_lower) :
    handle_(nh), upper_(port_upper), lower_(port_lower)
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

  // wheel
  ROS_INFO(" create wheel servo sub");
  wheel_servo_sub_ =
      handle_.subscribe(
          "wheel_servo",
          10,
          &AeroControllerNode::WheelServoCallback,
          this);
  ROS_INFO(" create wheel command sub");
  wheel_sub_ =
      handle_.subscribe(
          "wheel_command",
          10,
          &AeroControllerNode::WheelCommandCallback,
          this);

  // get state
  // if get_state == false, it will be passed.
  bool get_state = true;
  handle_.param<bool> ("get_state", get_state, true);
  if (get_state) {
    ROS_INFO(" create timer sub");
    timer_ =
        handle_.createTimer(ros::Duration(0.1),
                            &AeroControllerNode::JointStateCallback, this);
  } else {
    ROS_INFO(" controller DO NOT return state.");
    JointStateOnce();
  }
  ROS_INFO(" done");
}

AeroControllerNode::~AeroControllerNode() {
}

void AeroControllerNode::GoVelocityCallback(
    const geometry_msgs::Twist::ConstPtr& msg) {
  boost::mutex::scoped_lock lock(mtx_);
  lower_.flush();
  usleep(1000 * 10);
}

void AeroControllerNode::JointTrajectoryCallback(
    const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
  boost::mutex::scoped_lock lock(mtx_);

  // joint name to indeices, if not exist, then return -1
  std::vector<int32_t> upper_joint_to_stroke_indices;
  std::vector<int32_t> lower_joint_to_stroke_indices;
  for (size_t i = 0; i < msg->joint_names.size(); i++) {
    std::string joint_name = msg->joint_names[i];
    upper_joint_to_stroke_indices.push_back(
        upper_.get_stroke_index_from_joint_name(joint_name));
    lower_joint_to_stroke_indices.push_back(
        lower_.get_stroke_index_from_joint_name(joint_name));
  }

  // if joint stroke is undefined, use previous stroke
  std::vector<int16_t> upper_stroke_vector;
  std::vector<int16_t>& upper_ref_vector =
      upper_.get_reference_stroke_vector();
  upper_stroke_vector.assign(upper_ref_vector.begin(),
                             upper_ref_vector.end());

  std::vector<int16_t> lower_stroke_vector;
  std::vector<int16_t>& lower_ref_vector =
      lower_.get_reference_stroke_vector();
  lower_stroke_vector.assign(lower_ref_vector.begin(),
                             lower_ref_vector.end());

  // count axis in command, if 0, then command not send
  size_t upper_count = 0;
  size_t lower_count = 0;

  // for each trajectory points,
  for (size_t i = 0; i < msg->points.size(); i++) {
    // convert positions to stroke vector
    for (size_t j = 0; j < msg->points[i].positions.size(); j++) {
      // upper
      if (upper_joint_to_stroke_indices[j] >= 0) {
        upper_stroke_vector[
            static_cast<size_t>(upper_joint_to_stroke_indices[j])] =
            static_cast<int16_t>(100.0 * msg->points[i].positions[j]);
        upper_count++;
      }
      // lower
      if (lower_joint_to_stroke_indices[j] >= 0) {
        lower_stroke_vector[
            static_cast<size_t>(lower_joint_to_stroke_indices[j])] =
            static_cast<int16_t>(100.0 * msg->points[i].positions[j]);
        lower_count++;
      }
    }
    double time_sec = msg->points[i].time_from_start.toSec();
    uint16_t time_msec = static_cast<uint16_t>(time_sec * 1000.0);
    if (upper_count > 0) {
      upper_.flush();
      upper_.set_position(upper_stroke_vector, time_msec);
    }
    if (lower_count > 0) {
      lower_.flush();
      lower_.set_position(lower_stroke_vector, time_msec);
    }
    usleep(static_cast<int32_t>(time_sec * 1000.0 * 1000.0));
  }
}

void AeroControllerNode::JointStateCallback(const ros::TimerEvent& event) {
  JointStateOnce();
}

void AeroControllerNode::JointStateOnce() {
  boost::mutex::scoped_lock lock(mtx_);

  pr2_controllers_msgs::JointTrajectoryControllerState state;
  state.header.stamp = ros::Time::now();
  state.joint_names.resize(AERO_DOF);
  state.desired.positions.resize(AERO_DOF);
  state.actual.positions.resize(AERO_DOF);

  std::vector<int16_t> upper_stroke_vector;
  std::vector<int16_t>& upper_ref_vector =
      upper_.get_reference_stroke_vector();
  std::vector<int16_t> lower_stroke_vector;
  std::vector<int16_t>& lower_ref_vector =
      lower_.get_reference_stroke_vector();
  upper_.flush();
  upper_.get_position(upper_stroke_vector);
  lower_.flush();
  lower_.get_position(lower_stroke_vector);
  for (size_t i = 0; i < AERO_DOF_UPPER; i++) {
    state.joint_names[i] = upper_.get_joint_name(i);
    state.desired.positions[i] =
        static_cast<double>(upper_ref_vector[i]) * 0.01;
    state.actual.positions[i] =
        static_cast<double>(upper_stroke_vector[i]) * 0.01;
  }
  for (size_t i = 0; i < AERO_DOF_LOWER; i++) {
    state.joint_names[i + AERO_DOF_UPPER] = lower_.get_joint_name(i);
    state.desired.positions[i + AERO_DOF_UPPER] =
        static_cast<double>(lower_ref_vector[i]) * 0.01;
    state.actual.positions[i + AERO_DOF_UPPER] =
        static_cast<double>(lower_stroke_vector[i]) * 0.01;
  }
  state_pub_.publish(state);
}

///////////////////////////
// for wheel
void AeroControllerNode::WheelServoCallback(
    const std_msgs::Bool::ConstPtr& msg) {
  boost::mutex::scoped_lock lock(mtx_);

  if (msg->data) {
    // wheel_on means all joints and wheels servo on
    lower_.wheel_on();
  } else {
    // servo_on means only joints servo on, and wheels servo off
    lower_.servo_on();
  }
}
void AeroControllerNode::WheelCommandCallback(
    const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
  boost::mutex::scoped_lock lock(mtx_);

  // wheel name to indeices, if not exist, then return -1
  std::vector<int32_t> joint_to_wheel_indices;
  for (size_t i = 0; i < msg->joint_names.size(); i++) {
    std::string joint_name = msg->joint_names[i];
    joint_to_wheel_indices.push_back(
        lower_.get_wheel_index_from_wheel_name(joint_name));
  }

  // use previous velocity
  std::vector<int16_t> wheel_vector;
  std::vector<int16_t>& ref_vector =
      lower_.get_reference_wheel_vector();
  wheel_vector.assign(ref_vector.begin(), ref_vector.end());

  // for each trajectory points,
  for (size_t i = 0; i < msg->points.size(); i++) {
    // convert positions to stroke vector
    for (size_t j = 0; j < msg->points[i].positions.size(); j++) {
      if (joint_to_wheel_indices[j] >= 0) {
        wheel_vector[
            static_cast<size_t>(joint_to_wheel_indices[j])] =
            static_cast<int16_t>(msg->points[i].positions[j]);
      }
    }
    double time_sec = msg->points[i].time_from_start.toSec();
    uint16_t time_msec = static_cast<uint16_t>(time_sec * 1000.0);
    lower_.flush();
    lower_.set_wheel_velocity(wheel_vector, time_msec);
    usleep(static_cast<int32_t>(time_sec * 1000.0 * 1000.0));
  }
}



}  // namespace


int main(int argc, char** argv) {
  ros::init(argc, argv, "aero_controller");
  ros::NodeHandle np("~");

  std::string port_upper("/dev/aero_upper");
  std::string port_lower("/dev/aero_lower");

  bool debug = false;

  np.param<bool> ("debug", debug, false);

  if (debug) {
    port_upper = "";
    port_lower = "";
  } else {
    np.param<std::string> ("port_upper", port_upper, "/dev/aero_upper");
    np.param<std::string> ("port_lower", port_lower, "/dev/aero_lower");
  }

  ROS_INFO_STREAM("initializing ctrl with " <<
                  port_upper << " and " << port_lower);
  aero_controller::AeroControllerNode aero(np, port_upper, port_lower);
  ROS_INFO("configuring done");

  ros::spin();

  return 0;
}
