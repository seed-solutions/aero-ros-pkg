#include "aero_move_base/AeroMoveBase.hh"

using namespace aero;
using namespace navigation;

static const float radius = 0.076;
static const float Radius = 0.2974535;
// static const float max_velocity = 450.0;  // rpm * 10
static const float max_velocity = 90.0;  // deg/s

static const float ktheta = -5.54420;
static const float kv = 13.1579;

//////////////////////////////////////////////////
void AeroMoveBase::Init()
{
  ros_rate_ = 0.05;
  odom_rate_ = 0.02;
  safe_rate_ = 0.5;
  safe_duration_ = 1.0;
  num_of_wheels_ = 4;
  wheel_names_ =
    {"can_front_l_wheel", "can_front_r_wheel",
     "can_rear_l_wheel", "can_rear_r_wheel"};
}

//////////////////////////////////////////////////
void AeroMoveBase::VelocityToWheel(
    const geometry_msgs::TwistConstPtr& _cmd_vel,
    std::vector<double>& _wheel_vel)
{
  float dx, dy, dtheta, theta;
  float v1, v2, v3, v4;
  int16_t FR_wheel, RR_wheel, FL_wheel, RL_wheel;
  theta = 0.0;  // this means angle in local coords, so always 0

  float cos_theta = cos(theta);
  float sin_theta = sin(theta);

  // change dy and dx, because of between ROS and vehicle direction
  dy = (_cmd_vel->linear.x * cos_theta - _cmd_vel->linear.y * sin_theta);
  dx = (_cmd_vel->linear.x * sin_theta + _cmd_vel->linear.y * cos_theta);
  dtheta = _cmd_vel->angular.z;  // desirede angular velocity

  // calculate wheel velocity
  v1 = ktheta * dtheta +
      kv * ((-cos_theta + sin_theta) * dx + (-cos_theta - sin_theta) * dy);
  v2 = ktheta * dtheta +
      kv * ((-cos_theta - sin_theta) * dx + ( cos_theta - sin_theta) * dy);
  v3 = ktheta * dtheta +
      kv * (( cos_theta - sin_theta) * dx + ( cos_theta + sin_theta) * dy);
  v4 = ktheta * dtheta +
      kv * (( cos_theta + sin_theta) * dx + (-cos_theta + sin_theta) * dy);

  //[rad/sec] -> [deg/sec]
  FR_wheel = static_cast<int16_t>(v1 * (180 / M_PI));
  RR_wheel = static_cast<int16_t>(v4 * (180 / M_PI));
  FL_wheel = static_cast<int16_t>(v2 * (180 / M_PI));
  RL_wheel = static_cast<int16_t>(v3 * (180 / M_PI));

  _wheel_vel[0] = FL_wheel;
  _wheel_vel[1] = FR_wheel;
  _wheel_vel[2] = RL_wheel;
  _wheel_vel[3] = RR_wheel;
}
