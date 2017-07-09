#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

//////////////////////////////
class SetOdometry{
public:
  SetOdometry(ros::NodeHandle _nh);
  void CmdvelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel);
  void CalculateOdometry(const ros::TimerEvent& _event);

private:
  double vx_,vy_,vth_,x_,y_,th_;
  double dt_,delta_x_,delta_y_,delta_th_;

  ros::Publisher odom_pub_;
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;

  ros::Timer timer_;
  ros::Time current_time_, last_time_;

  geometry_msgs::Quaternion odom_quat_;
  tf::TransformBroadcaster odom_broadcaster_;
  geometry_msgs::TransformStamped odom_trans_;
  nav_msgs::Odometry odom_;
};
//-----------------------------

SetOdometry::SetOdometry(ros::NodeHandle _nh)
  :vx_(0),vy_(0),vth_(0),x_(0),y_(0),th_(0),
  dt_(0),delta_x_(0),delta_y_(0),delta_th_(0){
  nh_ = _nh;
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1000, &SetOdometry::CmdvelCallback,this);
  timer_ = nh_.createTimer(ros::Duration(0.02), &SetOdometry::CalculateOdometry, this);

  current_time_ = ros::Time::now();
  last_time_ = ros::Time::now();
}
//-------------------

void SetOdometry::CmdvelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel){
  vx_ = _cmd_vel->linear.x;
  vy_ = _cmd_vel->linear.y;
  vth_ = _cmd_vel->angular.z;
}

void SetOdometry::CalculateOdometry(const ros::TimerEvent& _event){

  current_time_ = ros::Time::now();
  dt_ = (current_time_ - last_time_).toSec();
  delta_x_ = (vx_ * cos(th_) - vy_ * sin(th_)) * dt_;
  delta_y_ = (vx_ * sin(th_) + vy_ * cos(th_)) * dt_;
  delta_th_ = vth_ * dt_;

  x_ += delta_x_;
  y_ += delta_y_;
  th_ += delta_th_;

  // odometry is 6DOF so we'll need a quaternion created from yaw
  odom_quat_ = tf::createQuaternionMsgFromYaw(th_);

  // first, we'll publish the transform over tf
  odom_trans_.header.stamp = current_time_;
  odom_trans_.header.frame_id = "odom";
  odom_trans_.child_frame_id = "base_link";

  odom_trans_.transform.translation.x = x_;
  odom_trans_.transform.translation.y = y_;
  odom_trans_.transform.translation.z = 0.0;
  odom_trans_.transform.rotation = odom_quat_;

  // send the transform
  odom_broadcaster_.sendTransform(odom_trans_);

  // next, we'll publish the odometry message over ROS
  odom_.header.stamp = current_time_;
  odom_.header.frame_id = "odom";

  // set the position
  odom_.pose.pose.position.x = x_;
  odom_.pose.pose.position.y = y_;
  odom_.pose.pose.position.z = 0.0;
  odom_.pose.pose.orientation = odom_quat_;

  // set the velocity
  odom_.child_frame_id = "base_link";
  odom_.twist.twist.linear.x = vx_;
  odom_.twist.twist.linear.y = vy_;
  odom_.twist.twist.angular.z = vth_;

  // publish the message
  odom_pub_.publish(odom_);

  last_time_ = current_time_;
}

/////////////////////////////////////////////////
int main( int argc, char** argv) {
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle nh;

  SetOdometry so(nh);

  ros::spin();
}
