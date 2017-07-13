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
  /// MUST be implemented in subclass.
 private: void Init();

 private: void MoveBase(const ros::TimerEvent& _event);

 private: bool MoveBaseOnce();

  /// @brief ABSTRACT function,
  /// returns command for each wheels from x, y, theta.
  ///
  /// This function depends on hardware construction and
  /// MUST be implemented in subclass.
 private: wheels Translate(float _x, float _y, float _theta);

  /// @brief ABSTRACT function,
  /// returns position from wheel velocities and dt
  ///
  /// This function depends on hardware construction and
  /// MUST be implemented in subclass.
 private: pose dX(std::vector<double> _vels, float _dt);

  /// @brief ABSTRACT function,
  /// convert velocity to wheel velocity (v0, ... vn)
 private: void VelocityToWheel(
     const geometry_msgs::TwistConstPtr& _cmd_vel,
     std::vector<double>& _wheel_vel);

 private: void SetSimpleGoal(
     const geometry_msgs::PoseStamped::ConstPtr& _msg);

 private: void SetActionGoal();

 private: void CancelGoal();

 private: void FinishMove();

 private: void SetGoal(float _x, float _y, float _theta);

  /// @brief control with cmd_vel
 private: void CmdVelCallback(const geometry_msgs::TwistConstPtr& _cmd_vel);

  /// @brief safety stopper when msg is not reached for a while
 private: void SafetyCheckCallback(const ros::TimerEvent& _event);

  /// @brief odometry publisher
 private: void CalculateOdometry(const ros::TimerEvent& _event);

 private: std::vector<std::string> wheel_names_;

 private: float ros_rate_;

 private: int num_of_wheels_;

 private: float warm_up_time_;

 private: unsigned int wait_for_servo_usec_;

 private: float move_coefficient_x;

 private: float move_coefficient_y;

 private: float move_coefficient_theta;

 private: goal goal_;

 private: states states_;

 private: ros::NodeHandle nh_;

 private: actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;

 private: move_base_msgs::MoveBaseFeedback feedback_;

  // @param wheel control publisher
 private: ros::Publisher wheel_pub_;

  // @param current wheel velocities
 private: std::vector<double> cur_vel_;

 private: trajectory_msgs::JointTrajectory wheel_cmd_;

 private: ros::Subscriber cmd_vel_sub_;

  /// @param current (x, y, theta) (vx, vy, vtheta)
 private: double vx_, vy_, vth_, x_, y_, th_;

 private: ros::Time current_time_, last_time_;

 private: geometry_msgs::Quaternion odom_quat_;

 private: tf::TransformBroadcaster odom_broadcaster_;

 private: geometry_msgs::TransformStamped odom_trans_;

 private: nav_msgs::Odometry odom_;

 private: ros::Publisher odom_pub_;

 private: ros::Timer odom_timer_;

 private: float odom_rate_;

  /// @param servo status
 private: std_msgs::Bool servo_;

  /// @param servo control publisher
 private: ros::Publisher servo_pub_;

 private: ros::Subscriber simple_goal_sub_;

 private: ros::Timer timer_;

 private: ros::Timer safe_timer_;

 private: float safe_rate_;

 private: ros::Time time_stamp_;

 private: class AeroMoveBaseImpl;

 private: std::shared_ptr<AeroMoveBaseImpl> impl_;
};

typedef std::shared_ptr<AeroMoveBase> AeroMoveBasePtr;

}  // navigation
}  // aero

#endif
