/*
 * This file auto-generated from script. Do not Edit!
 * Original : aero_description/typeF/controllers/AeroBaseController.hh
*/
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <stdint.h>

namespace aero {
  namespace navigation {

    static const float radius = 0.076;
    static const float Radius = 0.300822;
    // static const float max_velocity = 450.0;  // rpm * 10
    static const float max_velocity = 90.0;  // deg/s

    static const float ktheta = -5.58199;
    static const float kv = 13.1579;

    class AeroBaseConfig {
    public:
      AeroBaseConfig() {}
      ~AeroBaseConfig() {}
      void Init(double &_ros_rate,
                double &_odom_rate,
                double &_safe_rate,
                double &_safe_duration,
                int    &_num_of_wheels,
                std::vector< std::string > & _wheel_names) {
        _ros_rate  = 0.05;
        _odom_rate = 0.02;
        _safe_rate = 0.5;
        _safe_duration = 1.0;
        _num_of_wheels = 4;
        _wheel_names =
          {"can_front_l_wheel", "can_front_r_wheel",
           "can_rear_l_wheel", "can_rear_r_wheel"};
      }
      double get_ros_rate(void) { return 0.05; }
      double get_odom_rate(void) { return 0.02; }
      double get_safe_rate(void) { return 0.5; }
      double get_safe_duration(void) { return 1.0; }
      int    get_number_of_wheels(void) { return 4; }
      int get_wheel_names(std::vector< std::string > & _wheel_names) {
        _wheel_names =
          {"can_front_l_wheel", "can_front_r_wheel",
           "can_rear_l_wheel", "can_rear_r_wheel"};
      }
      void VelocityToWheel(double _linear_x, double _linear_y, double _angular_z,
                           std::vector<int16_t>& _wheel_vel) {
          float dx, dy, dtheta, theta;
          float v1, v2, v3, v4;
          int16_t FR_wheel, RR_wheel, FL_wheel, RL_wheel;
          theta = 0.0;  // this means angle in local coords, so always 0

          float cos_theta = cos(theta);
          float sin_theta = sin(theta);

          // change dy and dx, because of between ROS and vehicle direction
          dy = (_linear_x * cos_theta - _linear_y * sin_theta);
          dx = (_linear_x * sin_theta + _linear_y * cos_theta);
          dtheta = _angular_z;  // desirede angular velocity

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
    };
  }
}
