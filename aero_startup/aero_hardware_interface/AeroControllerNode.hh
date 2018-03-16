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
#include <algorithm>

#include "aero_hardware_interface/Constants.hh"
#include "aero_hardware_interface/AeroControllers.hh"

#include "aero_hardware_interface/AngleJointNames.hh"
#include "aero_hardware_interface/Stroke2Angle.hh"
#include "aero_hardware_interface/Angle2Stroke.hh"
#include "aero_hardware_interface/UnusedAngle2Stroke.hh"

#include "aero_hardware_interface/Interpolation.hh"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include "aero_startup/AeroInterpolation.h"
#include "aero_startup/AeroSendJoints.h"
#include "aero_startup/GraspControl.h"

#include <std_msgs/Float32.h>

#include <chrono>

namespace aero
{
  namespace time
  {
    inline std::chrono::high_resolution_clock::time_point now()
    { return std::chrono::high_resolution_clock::now(); };

    inline float ms(std::chrono::duration<double> _p)
    { return std::chrono::duration_cast<std::chrono::milliseconds>(_p).count();\
    };
  }
}

namespace aero
{
  namespace controller
  {

    /// @brief Handles kill running joint trajectory threads.
    struct thread_info
    {
      uint id;

      std::vector<int> joints;

      bool kill;
    };

    /// @brief Handles copy killed joint trajectory threads.
    struct killed_thread_info
    {
      std::vector<std::pair<std::vector<int16_t>, uint16_t> > trajectories;

      std::vector<aero::interpolation::InterpolationPtr> interpolation;

      int at_trajectory_num;

      int at_split_num;

      uint thread_id;

      float factor; // for speed overwrite
    };

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

      /// @brief joint tracjectory threads created from callback
      /// @param _interpolaiton information on how to interpolate
      /// @param _stroke_trajectory information of trajectory
      /// @param _trajectory_start_from skips trajectory
      /// @param _split_start_from skips split for first trajectory
      /// @param _factor current speed factor (for speed overwrite)
    private: void JointTrajectoryThread(
        std::vector<aero::interpolation::InterpolationPtr> _interpolation,
        std::vector<std::pair<std::vector<int16_t>, uint16_t> > _stroke_trajectory,
        int _trajectory_start_from,
        int _split_start_from, float _factor=1.0f);

    private: void LowerTrajectoryThread(
        std::vector<std::pair<std::vector<int16_t>, uint16_t> > _stroke_trajectory);

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

      /// @brief publish the information to /node_ns/in_action
      /// whether trajectories are in action or not.
    private: void PublishInAction(const ros::TimerEvent& event);

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

    //   /// @brief subscribe hand script message
    //   /// @param _msg true: grasp, false :ungrasp
    // private: void HandScriptCallback(
    //       const std_msgs::Int16MultiArray::ConstPtr& _msg);

    private: void StatusResetCallback(
	const std_msgs::Empty::ConstPtr& _msg);

    private: void SetCollisionModeCallback(
        const std_msgs::Int32::ConstPtr& _msg);

    private: bool SendJointsCallback(
        aero_startup::AeroSendJoints::Request &_req,
        aero_startup::AeroSendJoints::Response &_res);

    private: bool GetJointsCallback(
        aero_startup::AeroSendJoints::Request &_req,
        aero_startup::AeroSendJoints::Response &_res);

    private: bool GraspControlCallback(
        aero_startup::GraspControl::Request& _req,
        aero_startup::GraspControl::Response& _res);

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

    private: ros::Publisher status_pub_;

    private: ros::Publisher in_action_pub_;

    private: ros::Subscriber status_reset_sub_;

    private: ros::Subscriber collision_mode_set_sub_;

    private: ros::ServiceServer interpolation_server_;

    private: ros::ServiceServer send_joints_server_;

    private: ros::ServiceServer get_joints_server_;

    private: ros::ServiceServer grasp_control_server_;

    private: ros::Timer timer_;

    private: std::mutex mtx_upper_;

    private: std::mutex mtx_lower_;

      /// @brief saved interpolation settings
    private: std::vector<aero::interpolation::InterpolationPtr> interpolation_;

    private: std::mutex mtx_intrpl_;

      /// @brief info of on-going threads moving the upper body
      ///   used to kill threads when interfered
      ///   JointStateOnce does not update current position while threads > 0
    private: std::vector<thread_info> registered_threads_;

      /// @brief thread id count (increments every time new thread is created)
      ///   used for applying thread id to new thread
    private: uint global_thread_cnt_;

    private: std::mutex mtx_threads_;

      /// @brief info of killed thread moving the upper body
      ///   used to copy and re-split threads when interfered
    private: std::vector<killed_thread_info> thread_graveyard_;

    private: std::mutex mtx_thread_graveyard_;

    private: std::mutex mtx_lower_thread_;

    private: thread_info lower_thread_;

    private: killed_thread_info lower_killed_thread_info_;

      // @brief wether sendJoints is active or not
    private: bool send_joints_status_;

    private: std::mutex mtx_send_joints_status_;

    private: ros::Timer in_action_timer_;

      /// @brief 0:no abort, 1:abort and reset, 2:abort but external reset 
    private: int collision_abort_mode_;

      // for speed overwrite

    private: ros::SubscribeOptions speed_overwrite_ops_;

    private: ros::CallbackQueue speed_overwrite_queue_;

    private: ros::AsyncSpinner speed_overwrite_spinner_;

    private: ros::Subscriber speed_overwrite_sub_;

    private: std::mutex mtx_thread_postpone_;

    private: bool thread_postpone_;

    private: void SpeedOverwriteCallback(
	const std_msgs::Float32::ConstPtr& _msg);
    };

    typedef std::shared_ptr<AeroControllerNode> AeroControllerNodePtr;

  }
}

#endif
