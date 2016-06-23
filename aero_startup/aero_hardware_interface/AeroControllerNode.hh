#ifndef AERO_CONTROLLER_AERO_CONTROLLER_NODE_H_
#define AERO_CONTROLLER_AERO_CONTROLLER_NODE_H_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>

#include "aero_hardware_interface/Constants.hh"
#include "aero_hardware_interface/AeroControllers.hh"

#include "aero_hardware_interface/AngleJointNames.hh"
#include "aero_hardware_interface/Stroke2Angle.hh"
#include "aero_hardware_interface/Angle2Stroke.hh"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

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

    private: AeroUpperController upper_;

    private: AeroLowerController lower_;

    private: ros::NodeHandle nh_;

    // private: ros::Subscriber cmdvel_sub_;

    private: ros::Subscriber jointtraj_sub_;

    private: ros::Subscriber wheel_servo_sub_;

    private: ros::Subscriber wheel_sub_;

    private: ros::Subscriber util_sub_;

    private: ros::Publisher state_pub_;

    private: ros::Publisher stroke_state_pub_;

    private: ros::Timer timer_;

    private: boost::mutex mtx_;
    };

    typedef std::shared_ptr<AeroControllerNode> AeroControllerNodePtr;

  }
}

#endif
