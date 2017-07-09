#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include <math.h>

//////////////////////////////
class WheelController{
public:
  WheelController(ros::NodeHandle _nh);
  void SetAction(const geometry_msgs::TwistConstPtr& _cmd_vel);
  void SafetyCheckCallback(const ros::TimerEvent& _event);

private:
  //set_action
  std::vector<double> cur_vel_;
  float dx_,dy_,dtheta_,theta_,v1_,v2_,v3_,v4_;
  int16_t FR_wheel,RR_wheel,FL_wheel,RL_wheel;
  float ros_rate_;

  ros::Publisher wheel_pub_;
  trajectory_msgs::JointTrajectory wheel_cmd_;
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;

  //safety_check
  std_msgs::Bool servo_;
  ros::Publisher servo_pub_;
  ros::Timer timer_;
  ros::Time time_stamp_;
};
//-----------------------

WheelController::WheelController(ros::NodeHandle _nh)
  : dx_(0.0),dy_(0.0),dtheta_(0.0),theta_(0.0),v1_(0.0),v2_(0.0),v3_(0.0),v4_(0.0),
    FR_wheel(0),RR_wheel(0),FL_wheel(0),RL_wheel(0){

  nh_ = _nh;
  wheel_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/aero_controller/wheel_command", 10);
  servo_pub_ = nh_.advertise<std_msgs::Bool>("/aero_controller/wheel_servo", 10);
  cmd_vel_sub_ = nh_.subscribe("/cmd_vel",1, &WheelController::SetAction,this);
  timer_ = nh_.createTimer(ros::Duration(0.01), &WheelController::SafetyCheckCallback, this);

  ros_rate_ = 0.05;   //not meaning
  wheel_cmd_.joint_names = 
    {"can_front_l_wheel", "can_front_r_wheel",
     "can_rear_l_wheel", "can_rear_r_wheel"};

  cur_vel_.resize(4);
  wheel_cmd_.points.resize(1);

  servo_.data = false;

}
//-----------------------

void WheelController::SetAction(const geometry_msgs::TwistConstPtr& _cmd_vel){

  //check servo state
  if (servo_.data == false){
    servo_.data = true;
    servo_pub_.publish(servo_);
  }

  //change dy and dx, because of between ROS and vehicle direction
  dy_ = (_cmd_vel->linear.x * cos(theta_) - _cmd_vel->linear.y * sin(theta_));
  dx_ = (_cmd_vel->linear.x * sin(theta_) +_cmd_vel->linear.y * cos(theta_));
  dtheta_ = _cmd_vel->angular.z;                  //desirede angular velocity

  //calculate wheel velocity
  v1_ = -5.54420*dtheta_ + 13.1579*((-cos(theta_)+sin(theta_))*dx_ + (-cos(theta_)-sin(theta_))*dy_);
  v2_ = -5.54420*dtheta_ + 13.1579*((-cos(theta_)-sin(theta_))*dx_ + (cos(theta_)-sin(theta_))*dy_);
  v3_ = -5.54420*dtheta_ + 13.1579*((cos(theta_)-sin(theta_))*dx_ + (cos(theta_)+sin(theta_))*dy_);
  v4_ = -5.54420*dtheta_ + 13.1579*((cos(theta_)+sin(theta_))*dx_ + (-cos(theta_)+sin(theta_))*dy_);


  //[rad/sec] -> [deg/sec]
  FR_wheel = static_cast<int16_t>(v1_*(180/M_PI));
  RR_wheel = static_cast<int16_t>(v4_*(180/M_PI));
  FL_wheel = static_cast<int16_t>(v2_*(180/M_PI));
  RL_wheel = static_cast<int16_t>(v3_*(180/M_PI));

  cur_vel_[0] = FL_wheel;
  cur_vel_[1] = FR_wheel;
  cur_vel_[2] = RL_wheel;
  cur_vel_[3] = RR_wheel;

  wheel_cmd_.points[0].positions = cur_vel_;
  wheel_cmd_.points[0].time_from_start = ros::Duration(ros_rate_);
  wheel_pub_.publish(wheel_cmd_);

  //update time_stamp_
  time_stamp_ = ros::Time::now();
}
//---------------------------

void WheelController::SafetyCheckCallback(const ros::TimerEvent& _event){
  if( ((ros::Time::now() - time_stamp_).toSec() >= 1) && (servo_.data == true) ){
    cur_vel_[0] = 0;
    cur_vel_[1] = 0;
    cur_vel_[2] = 0;
    cur_vel_[3] = 0;

    wheel_cmd_.points[0].positions = cur_vel_;
    wheel_cmd_.points[0].time_from_start = ros::Duration(ros_rate_);
    wheel_pub_.publish(wheel_cmd_);

    servo_.data = false;
    servo_pub_.publish(servo_);
  }

} 

/////////////////////////////////////////////////
int main( int argc, char** argv) {
  ros::init(argc, argv, "wheel_controller");
  ros::NodeHandle nh;

  WheelController wc(nh);

  ros::spin();
}
