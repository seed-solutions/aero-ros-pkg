/// @author Sasabuchi Kazuhiro, Shintaro Hori, Hiroaki Yaguchi

#ifndef AERO_NAVIGATION_AERO_MOVE_BASE_H_
#define AERO_NAVIGATION_AERO_MOVE_BASE_H_

#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

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
 public: explicit AeroMoveBase(const ros::NodeHandle& _nh);

 public: ~AeroMoveBase();

  /// @brief ABSTRACT function, initialize wheel properties.
  ///
  /// This function depends on hardware construction and
  /// MUST be implemented in description directory.
 private: void Init();

  /// @brief ABSTRACT function,
  /// convert velocity to wheel velocity (v0, ... vn)
  ///
  /// This function depends on hardware construction and
  /// MUST be implemented in description directory.
 private: void VelocityToWheel(
     const geometry_msgs::TwistConstPtr& _cmd_vel,
     std::vector<double>& _wheel_vel);

 private: void CmdVelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel);

 private: void SafetyCheckCallback(const ros::TimerEvent& _event);

 private: void CalculateOdometry(const ros::TimerEvent& _event);

  /// @param names of wheel joints
 private: std::vector<std::string> wheel_names_;

  /// @param number of wheels
 private: int num_of_wheels_;

  /// @param rate for move base action
 private: float ros_rate_;

  /// @param node handle
 private: ros::NodeHandle nh_;

  /// @param wheel control publisher
 private: ros::Publisher wheel_pub_;

  /// @param current wheel velocities
 private: std::vector<double> cur_vel_;

  /// @param wheel control msg
 private: trajectory_msgs::JointTrajectory wheel_cmd_;

  /// @param subscriber for `cmd_vel`
 private: ros::Subscriber cmd_vel_sub_;

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
 private: float odom_rate_;

  /// @param servo status
 private: std_msgs::Bool servo_;

  /// @param servo control publisher
 private: ros::Publisher servo_pub_;


  /// @param timer for safety check
 private: ros::Timer safe_timer_;

  /// @param rate for safety check
 private: float safe_rate_;

  /// @param max duration for safety stop
 private: float safe_duration_;

  /// @param time stamp of the latest recieved cmd_vel msg
 private: ros::Time time_stamp_;

};

typedef std::shared_ptr<AeroMoveBase> AeroMoveBasePtr;

}  // navigation
}  // aero

#endif
