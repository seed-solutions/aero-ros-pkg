#include <aero_std/AeroLookatCommander.hh>

aero::lookat_commander::AeroLookatCommander::AeroLookatCommander(ros::NodeHandle &_nh,
                                                                 aero::interface::AeroMoveitInterface *_ami)
{
  tracking_mode_ = aero::tracking::disable;

  ami_ = _ami;

  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(ami_->kinematic_model));

  roll_max_vel_  = ami_->kinematic_model->getVariableBounds("neck_r_joint").max_velocity_;
  pitch_max_vel_ = ami_->kinematic_model->getVariableBounds("neck_p_joint").max_velocity_;
  yaw_max_vel_   = ami_->kinematic_model->getVariableBounds("neck_y_joint").max_velocity_;

  event_spinner_.reset(new ros::AsyncSpinner(1, &eventqueue_));
  sub_spinner_.reset(new ros::AsyncSpinner(1, &subqueue_));

  ros::TimerOptions tmopt(ros::Duration(0.07), // rate fixed by parameter
                          boost::bind(&aero::lookat_commander::AeroLookatCommander::timerCallback,
                                      this, _1),
                          &eventqueue_);
  event_timer_ = _nh.createTimer(tmopt);

  //
  event_spinner_->start();
  //sub_spinner_.start(); // ?
}

void aero::lookat_commander::AeroLookatCommander::disableTrackingMode()
{
  boost::mutex::scoped_lock lk(callback_mtx_);
  tracking_mode_ = aero::tracking::disable;
}

bool aero::lookat_commander::AeroLookatCommander::setTrackingMode(aero::tracking _mode, const aero::Vector3 &_pos)
{
  boost::mutex::scoped_lock lk(callback_mtx_);
  tracking_mode_ = _mode;

  switch(tracking_mode_) {
  case aero::tracking::map:
  case aero::tracking::map_static:
    {
      aero::Vector3 pos_in_base;
      pos_in_base = ami_->volatileTransformToBase(_pos.x(), _pos.y(), _pos.z());
      tracking_pos_ = _pos;
      sendNeckOnce_(pos_in_base);
    }
    break;
  case aero::tracking::base:
  case aero::tracking::base_static:
    tracking_pos_ = _pos;
    sendNeckOnce_(tracking_pos_);
    break;
  default:
    return false;
    break;
  }

  return true;
}

bool aero::lookat_commander::AeroLookatCommander::setNeckRPY(double _r, double _p, double _y)
{
  double prev_r, prev_p, prev_y;
  getNeckRPY(prev_r, prev_p, prev_y);

  ami_->setNeck_(_r, _p, _y, kinematic_state_);

  double time = std::max(std::max(std::abs(prev_y - _y) / yaw_max_vel_,
                                  std::abs(prev_p - _p) / pitch_max_vel_),
                         0.1);
  ROS_WARN("send neck %f", 1000*time);
  ami_->sendNeckAsync_(1000*time, kinematic_state_, 0.002);
  return true;
}

void aero::lookat_commander::AeroLookatCommander::getNeckRPY(double &_r, double &_p, double &_y)
{
  _r = kinematic_state_->getVariablePosition("neck_r_joint");
  _p = kinematic_state_->getVariablePosition("neck_p_joint");
  _y = kinematic_state_->getVariablePosition("neck_y_joint");
}

bool aero::lookat_commander::AeroLookatCommander::setLookAtTopic(const std::string &_topic)
{
  boost::mutex::scoped_lock lk(callback_mtx_);
  tracking_mode_ = aero::tracking::topic;

  // TODO:
  ros::SubscribeOptions sub_ops = ros::SubscribeOptions::create< geometry_msgs::Point >
    ( _topic, 1,
      boost::bind(&aero::lookat_commander::AeroLookatCommander::subCallback, this, _1),
      ros::VoidPtr(), &subqueue_);
  //sub_ = _nh.subscribe(sub_ops);

  return true;
}

void aero::lookat_commander::AeroLookatCommander::subCallback(const geometry_msgs::Point::ConstPtr &_msg) {
  //
}

void aero::lookat_commander::AeroLookatCommander::timerCallback(const ros::TimerEvent& ev)
{
  boost::mutex::scoped_lock lk(callback_mtx_);

  switch(tracking_mode_) {
  case aero::tracking::map:
    {
      aero::Vector3 pos_in_base;
      pos_in_base = ami_->volatileTransformToBase(tracking_pos_);
      sendNeckOnce_(pos_in_base);
    }
    break;
  case aero::tracking::base:
    {
      sendNeckOnce_(tracking_pos_);
    }
    break;
  default:
    return;
    break;
  }
}

void aero::lookat_commander::AeroLookatCommander::sendNeckOnce_(const aero::Vector3 &_pos)
{
  ami_->setRobotStateToCurrentState(kinematic_state_);
  double prev_r, prev_p, prev_y;
  getNeckRPY(prev_r, prev_p, prev_y);

  ami_->setLookAt_(_pos, kinematic_state_);

  double r, p, y;
  getNeckRPY(r, p, y);

  double time = std::max(std::max(std::abs(prev_y - y) / yaw_max_vel_,
                                  std::abs(prev_p - p) / pitch_max_vel_),
                         0.1);
  ROS_WARN("send neck %f", 1000*time);

  ami_->sendNeckAsync_(1000 * time, kinematic_state_, 0.002);
}
