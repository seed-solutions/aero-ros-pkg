/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi

#include "aero_move_base/AeroMoveBase.hh"

using namespace aero;
using namespace navigation;

//////////////////////////////////////////////////
/// @brief constructor
/// @param _nh ROS Node Handle
AeroMoveBase::AeroMoveBase(const ros::NodeHandle& _nh) :
  nh_(_nh),
  vx_(0), vy_(0), vth_(0), x_(0), y_(0), th_(0), base_config_()
{
  base_config_.Init(ros_rate_,
                    odom_rate_,
                    safe_rate_,
                    safe_duration_,
                    num_of_wheels_,
                    wheel_names_);

  wheel_cmd_.joint_names = wheel_names_;

  cur_vel_.resize(num_of_wheels_);
  wheel_cmd_.points.resize(1);

  servo_.data = false;

  current_time_ = ros::Time::now();
  last_time_ = current_time_;


  wheel_pub_ =
      nh_.advertise<trajectory_msgs::JointTrajectory>(
          "/aero_controller/wheel_command", 10);

  servo_pub_ =
      nh_.advertise<std_msgs::Bool>("/aero_controller/wheel_servo", 10);

  // for cmd_vel
  cmd_vel_sub_ =
      nh_.subscribe("/cmd_vel",1, &AeroMoveBase::CmdVelCallback, this);

  // for odometory
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

  odom_timer_ = nh_.createTimer(ros::Duration(odom_rate_),
                                &AeroMoveBase::CalculateOdometry, this);

  // for safety check
  safe_timer_ =
      nh_.createTimer(ros::Duration(safe_rate_),
                      &AeroMoveBase::SafetyCheckCallback, this);
}

//////////////////////////////////////////////////
/// @brief destructor
AeroMoveBase::~AeroMoveBase()
{
}

//////////////////////////////////////////////////
/// @brief control with cmd_vel
void AeroMoveBase::CmdVelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel)
{
  if (std::isnan(_cmd_vel->linear.x) ||
      std::isnan(_cmd_vel->linear.y) ||
      std::isnan(_cmd_vel->angular.z)) {
    ROS_ERROR("there is nan in /cmd_vel: ( %f %f %f )",
              _cmd_vel->linear.x, _cmd_vel->linear.x, _cmd_vel->angular.z);
    return;
  }
  vx_  = _cmd_vel->linear.x;
  vy_  = _cmd_vel->linear.y;
  vth_ = _cmd_vel->angular.z;

  //check servo state
  if (!servo_.data){
    servo_.data = true;
    servo_pub_.publish(servo_);
  }
  std::vector<int16_t> int_vel;
  int_vel.resize(cur_vel_.size());

  // convert velocity to wheel
  base_config_.VelocityToWheel(vx_, vy_, vth_, int_vel);
  for(int i = 0; i < cur_vel_.size(); i++) {
    cur_vel_[i] = static_cast<double>(int_vel[i]);
  }

  wheel_cmd_.points[0].positions = cur_vel_;
  wheel_cmd_.points[0].time_from_start = ros::Duration(ros_rate_);
  wheel_pub_.publish(wheel_cmd_);

  //update time_stamp_
  time_stamp_ = ros::Time::now();
}

//////////////////////////////////////////////////
/// @brief safety stopper when msg is not reached
///  for more than `safe_duration_` [s]
void AeroMoveBase::SafetyCheckCallback(const ros::TimerEvent& _event)
{
  if((ros::Time::now() - time_stamp_).toSec() >= safe_duration_ &&
     servo_.data) {
    for (size_t i = 0; i < num_of_wheels_; i++) {
      cur_vel_[i] = 0;
    }

    wheel_cmd_.points[0].positions = cur_vel_;
    wheel_cmd_.points[0].time_from_start = ros::Duration(ros_rate_);
    wheel_pub_.publish(wheel_cmd_);

    servo_.data = false;
    servo_pub_.publish(servo_);
  }
}

//////////////////////////////////////////////////
/// @brief odometry publisher
void AeroMoveBase::CalculateOdometry(const ros::TimerEvent& _event)
{
  current_time_ = ros::Time::now();

  double dt, delta_x, delta_y, delta_th;
  dt = (current_time_ - last_time_).toSec();
  delta_x = (vx_ * cos(th_) - vy_ * sin(th_)) * dt;
  delta_y = (vx_ * sin(th_) + vy_ * cos(th_)) * dt;
  delta_th = vth_ * dt;

  x_  += delta_x;
  y_  += delta_y;
  th_ += delta_th;

  if (std::isnan(x_) ||
      std::isnan(y_) ||
      std::isnan(th_)) {
    ROS_ERROR("there is nan in odom: ( %f %f %f )", x_, y_, th_);
  }

  // odometry is 6DOF so we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat
      = tf::createQuaternionMsgFromYaw(th_);

  // first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time_;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  // send the transform
  odom_broadcaster_.sendTransform(odom_trans);

  // next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time_;
  odom.header.frame_id = "odom";

  // set the position
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  // set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx_;
  odom.twist.twist.linear.y = vy_;
  odom.twist.twist.angular.z = vth_;

  // publish the message
  odom_pub_.publish(odom);

  last_time_ = current_time_;
}
