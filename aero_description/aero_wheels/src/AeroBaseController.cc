#include "aero_navigation/move_base/AeroMoveBase.hh"

using namespace aero;
using namespace navigation;

//////////////////////////////////////////////////
void AeroMoveBase::Init()
{
  ros_rate_ = 0.2;
  num_of_wheels_ = 4;
  wheel_names_ =
    {"can_front_l_wheel", "can_front_r_wheel",
     "can_rear_l_wheel", "can_rear_r_wheel"};
}

//////////////////////////////////////////////////
wheels AeroMoveBase::Translate(float _x, float _y)
{
  static const float max_velocity = 308.0;

  // front_left and rear_right
  std::function<float(float, float)> lambda_vel1 =
    [=](float x, float y)
    {
      float result;

      if (x > 0 && y > 0)
      {
	float theta = atan(y / x);
	result = max_velocity * 4/M_PI * (0.25*M_PI - theta);
      }
      else if (y >= 0 && x <= 0)
	result = -max_velocity;
      else if (y <= 0 && x >= 0)
	result = max_velocity;
      else
      {
	float theta = atan(y / x) - M_PI;
	result = max_velocity * 4/M_PI * (theta + 0.75*M_PI);
      }

      return result;
    };

  // front_right and rear_left
  std::function<float(float, float)> lambda_vel2 =
    [=](float x, float y)
    {
      float result;

      if (x >= 0 && y >= 0)
	result = max_velocity;
      else if (y > 0 && x < 0)
      {
	float theta = M_PI - atan(y / x);
	result = max_velocity * 4/M_PI * (0.75*M_PI - theta);
      }
      else if (y < 0 && x > 0)
      {
	float theta = atan(y / x);
	result = max_velocity * 4/M_PI * (theta + 0.25*M_PI);
      }
      else
	result = -max_velocity;

      return result;
    };

  std::vector<float> velocities =
    {lambda_vel1(_x, _y), lambda_vel2(_x, _y),
     lambda_vel2(_x, _y), lambda_vel1(_x ,_y)};

  static const float radius = 100.0;

  float Vx =
    0.25 * 0.5 *
    (velocities[0] + velocities[1] + velocities[2] + velocities[3]);
  float Vy =
    0.25 * 0.5 *
    (-velocities[0] + velocities[1] + velocities[2] - velocities[3]);

  wheels wheel_data;
  wheel_data.velocities = // move forward positive to signal positive
    {-velocities[0], velocities[1], -velocities[2], velocities[3]};
  wheel_data.time = sqrt(_x*_x + _y*_y) / (sqrt(Vx*Vx + Vy*Vy) * radius) * 60;

  return wheel_data;
}

//////////////////////////////////////////////////
wheels AeroMoveBase::Rotate(float _theta)
{
  static const float max_velocity = 308.0;
  static const float radius = 100.0;
  static const float Radius = 254.56;

  wheels wheel_data;
  if (_theta >= 0)
    wheel_data.velocities =
      {max_velocity, max_velocity,
       max_velocity, max_velocity};
  else
    wheel_data.velocities =
      {-max_velocity, -max_velocity,
       -max_velocity, -max_velocity};
  wheel_data.time = fabs(_theta) * sqrt(2) * Radius /
    (max_velocity * radius) * 60;

  return wheel_data;
}

//////////////////////////////////////////////////
pose AeroMoveBase::dX(std::vector<double> _vels, float _dt)
{
  if (_vels.size() != 4) return {0.0, 0.0, 0.0};

  std::vector<float> velocities = // signal positive to move forward positive
    {-_vels[0], _vels[1], -_vels[2], _vels[3]};

  float Vx =
    0.25 * 0.5 *
    (velocities[0] + velocities[1] + velocities[2] + velocities[3]);
  float Vy =
    0.25 * 0.5 *
    (-velocities[0] + velocities[1] + velocities[2] - velocities[3]);

  static const float radius = 100.0;
  static const float max_velocity = 308.0;
  static const float Radius = 254.56;

  return {Vx*radius*_dt, Vy*radius*_dt,
      max_velocity * radius * _dt / (sqrt(2) * Radius)};
}
