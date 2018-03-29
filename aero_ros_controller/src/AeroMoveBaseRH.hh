/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi

#ifndef AERO_NAVIGATION_AERO_MOVE_BASE_H_
#define AERO_NAVIGATION_AERO_MOVE_BASE_H_

#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

//#include <actionlib/server/simple_action_server.h>
//#include <move_base_msgs/MoveBaseAction.h>
//#include <move_base_msgs/MoveBaseFeedback.h>
//#include <move_base_msgs/MoveBaseResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "aero_move_base/AeroBaseController.hh" // class AeroBaseConfig
#include "aero_robot_hardware.h"

namespace aero
{
namespace navigation
{

/// @brief wheel velocities and goal time
struct wheels
{
  std::vector<float> velocities;

  float time;
};

/// @brief 2D pose
struct pose
{
  float x;

  float y;

  float theta;
};

struct goal
{
  std::vector<float> max_vel;

  std::vector<float> wheel_dV;

  float run_time;

  float warm_up_time;
};

struct states
{
  std::vector<double> cur_vel;

  float cur_time;

  pose moved_distance;

  bool wheel_on;
};

/// @brief Base class of base movement
///
/// This class provides prototype of move base functions for
/// vehicle-type base.
/// Implementations of each hardwares must locate under
/// aero_description/{hardware_type}.
/// aero_description/aero_wheels/controllers/AeroBaseControllers.cc
/// is sample of implementation.
class AeroMoveBase
{
 public: explicit AeroMoveBase(const ros::NodeHandle& _nh,
                               aero_robot_hardware::AeroRobotHW *_in_hw);

 public: ~AeroMoveBase();

 private: void CmdVelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel);

 private: void SafetyCheckCallback(const ros::TimerEvent& _event);

 private: void CalculateOdometry(const ros::TimerEvent& _event);

  /// @param node handle
 private: ros::NodeHandle nh_;

  /// @param names of wheel joints
 private: std::vector<std::string> wheel_names_;

  /// @param number of wheels
 private: int num_of_wheels_;

  /// @param rate for move base action
 private: double ros_rate_;

  /// @param subscriber for `cmd_vel`
 private: ros::Subscriber cmd_vel_sub_;
 private: ros::CallbackQueue base_queue_;
 private: ros::AsyncSpinner base_spinner_;
 private: ros::SubscribeOptions base_ops_;

  /// @param current (x, y, theta) (vx, vy, vtheta)
 private: double vx_, vy_, vth_, x_, y_, th_;

  /// @param storing current and last time when msg recieved
 private: ros::Time current_time_, last_time_;

  /// @param tf broadcaster for odom
 private: tf::TransformBroadcaster odom_broadcaster_;

  /// @param odom publisher
 private: ros::Publisher odom_pub_;

  /// @param timer for odom
 private: ros::Timer odom_timer_;

  /// @param rate for odom
 private: double odom_rate_;

  /// @param servo status
 private: bool servo_;

  /// @param timer for safety check
 private: ros::Timer safe_timer_;

  /// @param rate for safety check
 private: double safe_rate_;

  /// @param max duration for safety stop
 private: double safe_duration_;

  /// @param time stamp of the latest recieved cmd_vel msg
 private: ros::Time time_stamp_;

 private: AeroBaseConfig base_config_;

  ///
 private: aero_robot_hardware::AeroRobotHW *hw_;

};

typedef std::shared_ptr<AeroMoveBase> AeroMoveBasePtr;

}  // navigation
}  // aero

#endif
