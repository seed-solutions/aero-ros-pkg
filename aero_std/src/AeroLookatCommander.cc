#include <aero_std/AeroLookatCommander.hh>

aero::lookat_commander::AeroLookatCommander::AeroLookatCommander(ros::NodeHandle &_nh,
                                                                 aero::interface::AeroMoveitInterface *_ami)
{
  tracking_mode_ = aero::tracking::disable;
  tracking_tf_  = "";
  tracking_pos_ = aero::Vector3(0, 0, 0);

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
  ROS_DEBUG("LookAt: disableTrackingMode");
  boost::mutex::scoped_lock lk(callback_mtx_);
  tracking_mode_ = aero::tracking::disable;
  // wait neck ...
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
      ROS_DEBUG("LookAt: setTrackingMode: volatile: %f %f %f (%f %f %f on map)",
                pos_in_base.x(), pos_in_base.y(), pos_in_base.z(),
                tracking_pos_.x(), tracking_pos_.y(), tracking_pos_.z());
      sendNeckOnce_(pos_in_base);
    }
    break;
  case aero::tracking::base:
  case aero::tracking::base_static:
    tracking_pos_ = _pos;
    ROS_DEBUG("LookAt: setTrackingMode: %f %f %f", _pos.x(), _pos.y(), _pos.z());
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
  ROS_DEBUG("LookAt: current neck (%f %f %f)", prev_r, prev_p, prev_y);

  ami_->setNeck_(_r, _p, _y, kinematic_state_);

  double time = std::max(std::max(std::abs(prev_y - _y) / yaw_max_vel_,
                                  std::abs(prev_p - _p) / pitch_max_vel_),
                         0.1);
  ROS_DEBUG("LookAt: sendNeckRPY (%f %f %f) / %f ms",
            _r, _p, _y, 1000*time);
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
  ROS_ERROR("LookAt: setLookAtTopic (not implemented yet)");
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

bool aero::lookat_commander::AeroLookatCommander::setLookAtTf(const std::string &_tf, bool _once)
{
  boost::mutex::scoped_lock lk(callback_mtx_);

  tracking_tf_ = _tf;

  {
    aero::Transform trans_in_base;
    ros::Time tm(0);
    ami_->listenTf(trans_in_base, "/base_link", tracking_tf_, tm);
    ROS_DEBUG_STREAM("LookAt: set tf: " << tracking_tf_ << " / " << trans_in_base);
    sendNeckOnce_(trans_in_base.translation());
  }

  if (!_once) {
    tracking_mode_ = aero::tracking::tf;
  }

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
      ROS_DEBUG("LookAt: volatile: %f %f %f (%f %f %f on map)",
                pos_in_base.x(), pos_in_base.y(), pos_in_base.z(),
                tracking_pos_.x(), tracking_pos_.y(), tracking_pos_.z());
      sendNeckOnce_(pos_in_base);
    }
    break;
  case aero::tracking::base:
    {
      sendNeckOnce_(tracking_pos_);
    }
    break;
  case aero::tracking::tf:
    {
      aero::Transform trans_in_base;
      ros::Time tm(0);
      ami_->listenTf(trans_in_base, "/base_link", tracking_tf_, tm);
      ROS_DEBUG_STREAM("LookAt: tf: " << tracking_tf_ << " / " << trans_in_base);
      sendNeckOnce_(trans_in_base.translation());
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
  ROS_DEBUG("LookAt: current neck (%f %f %f)",
            prev_r, prev_p, prev_y);

  ami_->setLookAt_(_pos, kinematic_state_);

  double r, p, y;
  getNeckRPY(r, p, y);
  ROS_DEBUG("LookAt: target neck (%f %f %f)", r, p, y);

  double time = std::max(std::max(std::abs(prev_y - y) / yaw_max_vel_,
                                  std::abs(prev_p - p) / pitch_max_vel_),
                         0.1);
  ROS_DEBUG("LookAt: sendNeck (%f %f %f) / %f ms",
            _pos.x(), _pos.y(), _pos.z(), 1000*time);

  ami_->sendNeckAsync_(1000 * time, kinematic_state_, 0.002);
}
