#ifndef AERO_CONTROLLER_AERO_CONTROLLER_NODE_H_
#define AERO_CONTROLLER_AERO_CONTROLLER_NODE_H_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>
#include <cmath>
#include <thread>
#include <mutex>

#include "aero_hardware_interface/Constants.hh"
#include "aero_hardware_interface/AeroControllers.hh"

#include "aero_hardware_interface/AngleJointNames.hh"
#include "aero_hardware_interface/Stroke2Angle.hh"
#include "aero_hardware_interface/Angle2Stroke.hh"
#include "aero_hardware_interface/UnusedAngle2Stroke.hh"

#include "aero_hardware_interface/Interpolation.hh"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "aero_startup/AeroInterpolation.h"

namespace aero
{
  namespace controller
  {

  /// @brief Aero controller node,
  /// has AeroUpperController and AeroLowerController
    class AeroControllerNode
    {
      /// @brief constructor
      /// @param _nh Node handle
      /// @param _port_upper Upper body USB port file name
      /// @param _port_lower Lower body USB port file name
    public:
      explicit AeroControllerNode(const ros::NodeHandle& _nh,
				  const std::string& _port_upper,
				  const std::string& _port_lower);
      /// @brief destructor
    public: ~AeroControllerNode();

    // private: void GoVelocityCallback(
    //     const geometry_msgs::Twist::ConstPtr& _msg);

      /// @brief subscribe joint tracjectory
      /// @param _msg joint trajectory
    private: void JointTrajectoryCallback(
        const trajectory_msgs::JointTrajectory::ConstPtr& _msg);

      /// @brief publish joint state, called by timer,
      /// main routine is implemented in JointStateOnce().
      /// @param _event timer event
    private: void JointStateCallback(const ros::TimerEvent& _event);

      /// @brief read joint state
    private: void JointStateOnce();

      /// @brief subscribe wheel servo message
      /// @param _msg true: on, false :off
    private: void WheelServoCallback(
        const std_msgs::Bool::ConstPtr& _msg);

      /// @brief subscribe wheel command message
      /// @param _msg wheel trajectory
    private: void WheelCommandCallback(
	const trajectory_msgs::JointTrajectory::ConstPtr& _msg);

      /// @brief subscribe utility servo message
      /// @param _msg true: on, false :off
    private: void UtilServoCallback(
        const std_msgs::Int32::ConstPtr& _msg);

      /// @brief server interpolation request message
      /// @param _req request interpolation method
      /// @param _res respond feedback of status
    private: bool InterpolationCallback(
        aero_startup::AeroInterpolation::Request &_req,
        aero_startup::AeroInterpolation::Response &_res);

    private: AeroUpperController upper_;

    private: AeroLowerController lower_;

    private: ros::NodeHandle nh_;

    // private: ros::Subscriber cmdvel_sub_;

    private: ros::Subscriber jointtraj_sub_;

    private: ros::SubscribeOptions jointtraj_ops_;

    private: ros::CallbackQueue jointtraj_queue_;

    private: ros::AsyncSpinner jointtraj_spinner_;

    private: ros::Subscriber wheel_servo_sub_;

    private: ros::SubscribeOptions wheel_ops_;

    private: ros::CallbackQueue wheel_queue_;

    private: ros::AsyncSpinner wheel_spinner_;

    private: ros::Subscriber wheel_sub_;

    private: ros::Subscriber util_sub_;

    private: ros::Publisher state_pub_;

    private: ros::Publisher stroke_state_pub_;

    private: ros::ServiceServer interpolation_server_;

    private: ros::Timer timer_;

    private: std::mutex mtx_upper_;

    private: std::mutex mtx_lower_;

      /// @brief saved interpolation settings
    private: std::vector<aero::interpolation::InterpolationPtr> interpolation_;

    private: std::mutex mtx_intrpl_;

      /// @brief count of on-going threads moving the upper body
      ///   JointStateOnce does not update current position while on_move_ > 0
    private: int on_move_;

    private: std::mutex mtx_on_move_cnt_;
    };

    typedef std::shared_ptr<AeroControllerNode> AeroControllerNodePtr;

  }
}

#endif
