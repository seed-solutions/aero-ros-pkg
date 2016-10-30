#include "aero_hardware_interface/AeroControllerNode.hh"

using namespace aero;
using namespace controller;

//////////////////////////////////////////////////
AeroControllerNode::AeroControllerNode(const ros::NodeHandle& _nh,
                                       const std::string& _port_upper,
                                       const std::string& _port_lower) :
    nh_(_nh), upper_(_port_upper), lower_(_port_lower)
{
  ROS_INFO("starting aero_hardware_interface");

  ROS_INFO(" create publisher");
  state_pub_ =
      nh_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(
          "state", 10);

  ROS_INFO(" create stroke publisher");
  stroke_state_pub_ =
      nh_.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>(
          "stroke_state", 10);

  // ROS_INFO(" create cmdvel sub");
  // cmdvel_sub_ =
  //     nh_.subscribe(
  //         "cmd_vel",
  //         10,
  //         &AeroControllerNode::GoVelocityCallback,
  //         this);

  ROS_INFO(" create command sub");
  jointtraj_sub_ =
      nh_.subscribe(
          "command",
          10,
          &AeroControllerNode::JointTrajectoryCallback,
          this);

  ROS_INFO(" create wheel servo sub");
  wheel_servo_sub_ =
      nh_.subscribe(
          "wheel_servo",
          10,
          &AeroControllerNode::WheelServoCallback,
          this);

  ROS_INFO(" create wheel command sub");
  wheel_sub_ =
      nh_.subscribe(
          "wheel_command",
          10,
          &AeroControllerNode::WheelCommandCallback,
          this);

  ROS_INFO(" create utility servo sub");
  util_sub_ =
      nh_.subscribe(
          "util_servo",
          10,
          &AeroControllerNode::UtilServoCallback,
          this);

  bool get_state = true;
  nh_.param<bool> ("get_state", get_state, true);

  if (get_state) {
    ROS_INFO(" create timer sub");
	//shiigi 0.1 -> 0.01
    timer_ =
        nh_.createTimer(ros::Duration(0.01),
                            &AeroControllerNode::JointStateCallback, this);
  } else {
    ROS_INFO(" controller DO NOT return state.");
    this->JointStateOnce();
  }

  ROS_INFO(" done");
}

//////////////////////////////////////////////////
AeroControllerNode::~AeroControllerNode()
{
}

//////////////////////////////////////////////////
// void AeroControllerNode::GoVelocityCallback(
//     const geometry_msgs::Twist::ConstPtr& _msg)
// {
//   boost::mutex::scoped_lock lock(mtx_);
//   lower_.flush();
//   usleep(1000 * 10);
// }

//////////////////////////////////////////////////
void AeroControllerNode::JointTrajectoryCallback(
    const trajectory_msgs::JointTrajectory::ConstPtr& _msg)
{
  boost::mutex::scoped_lock lock(mtx_);

  int number_of_angle_joints =
      upper_.get_number_of_angle_joints() +
      lower_.get_number_of_angle_joints();

  if (_msg->joint_names.size() > number_of_angle_joints)
    // invalid number of joints from _msg
    return;

  // positions in _msg are not ordered
  std::vector<int32_t> id_in_msg_to_ordered_id;
  id_in_msg_to_ordered_id.resize(_msg->joint_names.size());

  // if count is 0, commands will not go to robot
  //   used for occasions such as controlling only the upper body
  //   this way, controlling is safer
  size_t upper_count = 0;
  size_t lower_count = 0;

  // check for unused joints
  std::vector<bool> send_true(number_of_angle_joints, false);

  for (size_t i = 0; i < _msg->joint_names.size(); ++i) {
    // try finding name from upper_
    id_in_msg_to_ordered_id[i] =
        upper_.get_ordered_angle_id(_msg->joint_names[i]);

    // if finding name from upper_ failed
    if (id_in_msg_to_ordered_id[i] < 0) {
      // try finding name from lower_
      id_in_msg_to_ordered_id[i] =
        lower_.get_ordered_angle_id(_msg->joint_names[i]);
    } else {
      ++upper_count;
      send_true[id_in_msg_to_ordered_id[i]] = true;
      continue;
    }

    if (id_in_msg_to_ordered_id[i] < 0) return; // invalid name

    ++lower_count;
    send_true[id_in_msg_to_ordered_id[i]] = true;
  }

  // initiate trajectories

  std::vector<std::pair<std::vector<int16_t>, uint16_t> > upper_stroke_trajectory;

  // note: only upper will have interpolation
  if (upper_count > 0) {
    upper_stroke_trajectory.reserve(_msg->points.size() + 1);
    // get current stroke values
    std::vector<int16_t> ref_strokes;
    upper_.get_position(ref_strokes);
    // fill in unused joints to no-send
    for (size_t i = 0; i < ref_strokes.size(); ++i)
      if (!send_true[i]) ref_strokes[i] = 0x7fff;
    upper_stroke_trajectory.push_back({ref_strokes, 0});
  }

  // from here, get ready to handle the _msg positions

  // this is a tmp variable that is reused
  std::vector<double> ordered_positions(number_of_angle_joints);

  // for each trajectory points,
  for (size_t i = 0; i < _msg->points.size(); ++i) {
    std::fill(ordered_positions.begin(), ordered_positions.end(), 0.0);

    for (size_t j = 0; j < _msg->points[i].positions.size(); ++j)
      ordered_positions[id_in_msg_to_ordered_id[j]] =
        _msg->points[i].positions[j];

    std::vector<int16_t> strokes(AERO_DOF);
    common::Angle2Stroke(strokes, ordered_positions);

    // fill in unused joints to no-send
    for (size_t i = 0; i < strokes.size(); ++i)
      if (!send_true[i]) strokes[i] = 0x7fff;

    // split strokes into upper and lower
    std::vector<int16_t> upper_stroke_vector(
	strokes.begin(), strokes.begin() + AERO_DOF_UPPER);
    std::vector<int16_t> lower_stroke_vector(
	strokes.begin() + AERO_DOF_UPPER, strokes.end());

    double time_sec = _msg->points[i].time_from_start.toSec();
    uint16_t time_msec = static_cast<uint16_t>(time_sec * 100.0);

    if (upper_count > 0) {
      upper_stroke_trajectory.push_back({upper_stroke_vector, time_msec});
    }

    // only the first lower trajectory point is valid
    if (lower_count > 0 && i == 0) {
      lower_.flush();
      lower_.set_position(lower_stroke_vector, time_msec);
    }
  }

  if (upper_count <= 0) return; // nothing more to do

  std::vector<aero::interpolation::InterpolationPtr> interpolation;
  interpolation.reserve(upper_stroke_trajectory.size());
  // copy interpolation setup
  for (auto it = interpolation_.begin(); it != interpolation_.end(); ++it)
    interpolation.push_back(*it);
  // fillin rest with linear if not specified
  for (size_t i = interpolation_.size(); i < upper_stroke_trajectory.size(); ++i)
    interpolation.push_back(std::shared_ptr<aero::interpolation::Interpolation>(
        new aero::interpolation::Interpolation(aero::interpolation::i_linear)));

  if (interpolation_.size() < upper_stroke_trajectory.size());

  std::thread send_av([&](std::vector<aero::interpolation::InterpolationPtr>
                          _interpolation) {
      uint16_t msec_per_frame = 50; // 20 fps

      for (auto it = upper_stroke_trajectory.begin() + 1;
           it != upper_stroke_trajectory.end(); ++it) {
        // any movement faster than 600ms will not interpolate = linear
        if (it->second < 600) {
          upper_.flush();
          upper_.set_position(it->first, it->second);
          continue;
        }
        // find number of splits in this trajectory
        // time becomes slightly faster if not cleanly dividable
        int splits = static_cast<int>(it->second / msec_per_frame);
        int k = static_cast<int>(it - upper_stroke_trajectory.begin());
        // send splitted stroke
        for (size_t j = 0; j < splits; ++j) {
          float t_param = _interpolation.at(k)->interpolate(
              static_cast<float>(j) / splits);
          // calculate stroke in this split
          std::vector<int16_t> stroke(it->first.size());
          for (size_t i = 0; i < it->first.size(); ++i) {
            if (it->first[i] == 0x7fff) continue; // skip non-send joints
            stroke[i] =
              (1 - t_param) * (it - 1)->first[i] + t_param * it->first[i];
          }
          upper_.flush();
          // slightly longer time added for trajectory smoothness
          upper_.set_position(stroke, msec_per_frame + 10);
          usleep(static_cast<int32_t>(msec_per_frame * 1000.0 * 1000.0));
        }
      }
  }, interpolation);

  send_av.detach();
}

//////////////////////////////////////////////////
void AeroControllerNode::JointStateCallback(const ros::TimerEvent& event)
{
  this->JointStateOnce();
}

//////////////////////////////////////////////////
void AeroControllerNode::JointStateOnce()
{
  boost::mutex::scoped_lock lock(mtx_);

  // get desired positions
  std::vector<int16_t>& upper_ref_vector =
      upper_.get_reference_stroke_vector();
  std::vector<int16_t>& lower_ref_vector =
      lower_.get_reference_stroke_vector();

  // get upper actual positions
  upper_.flush();
  std::vector<int16_t> upper_stroke_vector;
  upper_.get_position(upper_stroke_vector);

  // get lower actual positions
  lower_.flush();
  std::vector<int16_t> lower_stroke_vector;
  lower_.get_position(lower_stroke_vector);

  // first handle the stroke publisher
  // in this way, we don't have to concatenate new vectors
  pr2_controllers_msgs::JointTrajectoryControllerState stroke_state;
  stroke_state.header.stamp = ros::Time::now();
  stroke_state.joint_names.resize(AERO_DOF);
  stroke_state.desired.positions.resize(AERO_DOF);
  stroke_state.actual.positions.resize(AERO_DOF);

  for (size_t i = 0; i < AERO_DOF_UPPER; ++i) {
    stroke_state.joint_names[i] = upper_.get_stroke_joint_name(i);
    stroke_state.desired.positions[i] =
        static_cast<double>(upper_ref_vector[i]);
    stroke_state.actual.positions[i] =
        static_cast<double>(upper_stroke_vector[i]);
  }
  for (size_t i = 0; i < AERO_DOF_LOWER; ++i) {
    stroke_state.joint_names[i + AERO_DOF_UPPER] =
        lower_.get_stroke_joint_name(i);
    stroke_state.desired.positions[i + AERO_DOF_UPPER] =
        static_cast<double>(lower_ref_vector[i]);
    stroke_state.actual.positions[i + AERO_DOF_UPPER] =
        static_cast<double>(lower_stroke_vector[i]);
  }

  int number_of_angle_joints =
      upper_.get_number_of_angle_joints() +
      lower_.get_number_of_angle_joints();

  pr2_controllers_msgs::JointTrajectoryControllerState state;
  state.header.stamp = ros::Time::now();
  state.joint_names.resize(number_of_angle_joints);
  state.desired.positions.resize(number_of_angle_joints);
  state.actual.positions.resize(number_of_angle_joints);

  // convert to desired angle positions
  std::vector<int16_t> desired_strokes(stroke_state.desired.positions.begin(),
				       stroke_state.desired.positions.end());
  common::Stroke2Angle(state.desired.positions, desired_strokes);

  // convert to actual angle positions
  std::vector<int16_t> actual_strokes(stroke_state.actual.positions.begin(),
				      stroke_state.actual.positions.end());
  common::Stroke2Angle(state.actual.positions, actual_strokes);

  // get joint names (auto-generated function)
  common::AngleJointNames(state.joint_names);

  state_pub_.publish(state);
  stroke_state_pub_.publish(stroke_state);
}

//////////////////////////////////////////////////
void AeroControllerNode::WheelServoCallback(
    const std_msgs::Bool::ConstPtr& _msg)
{
  boost::mutex::scoped_lock lock(mtx_);

  if (_msg->data) {
    // wheel_on sets all joints and wheels to servo on
    lower_.wheel_on();
  } else {
    // servo_on joints only, and servo off wheels
    lower_.servo_on();
  }
}

//////////////////////////////////////////////////
void AeroControllerNode::WheelCommandCallback(
    const trajectory_msgs::JointTrajectory::ConstPtr& _msg)
{
  boost::mutex::scoped_lock lock(mtx_);

  // wheel name to indices, if not exist, then return -1
  std::vector<int32_t> joint_to_wheel_indices(AERO_DOF_WHEEL);
  for (size_t i = 0; i < _msg->joint_names.size(); ++i) {
    std::string joint_name = _msg->joint_names[i];
    joint_to_wheel_indices[i] =
        lower_.get_wheel_id(joint_name);
  }

  // set previous velocity
  std::vector<int16_t> wheel_vector;
  std::vector<int16_t>& ref_vector =
      lower_.get_reference_wheel_vector();
  wheel_vector.assign(ref_vector.begin(), ref_vector.end());

  // for each trajectory points,
  for (size_t i = 0; i < _msg->points.size(); ++i) {
    // convert positions to stroke vector
    for (size_t j = 0; j < _msg->points[i].positions.size(); ++j) {
      if (joint_to_wheel_indices[j] >= 0) {
        wheel_vector[
            static_cast<size_t>(joint_to_wheel_indices[j])] =
            static_cast<int16_t>(_msg->points[i].positions[j]);
      }
    }

    double time_sec = _msg->points[i].time_from_start.toSec();
    uint16_t time_msec = static_cast<uint16_t>(time_sec * 100.0);
    lower_.flush();
    lower_.set_wheel_velocity(wheel_vector, time_msec);
    // usleep(static_cast<int32_t>(time_sec * 1000.0 * 1000.0));
  }
}

void AeroControllerNode::UtilServoCallback(
    const std_msgs::Int32::ConstPtr& _msg)
{
  boost::mutex::scoped_lock lock(mtx_);

  if (_msg->data == 0) {
    usleep(static_cast<int32_t>(200.0 * 1000.0));
    upper_.util_servo_off();
    usleep(static_cast<int32_t>(200.0 * 1000.0));
  } else {
    usleep(static_cast<int32_t>(200.0 * 1000.0));
    upper_.util_servo_on();
    usleep(static_cast<int32_t>(200.0 * 1000.0));
  }
}
