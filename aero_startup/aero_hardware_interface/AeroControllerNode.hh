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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace aero
{
  namespace controller
  {

    class AeroControllerNode
    {
    public:
      explicit AeroControllerNode(const ros::NodeHandle& _nh,
				  const std::string& _port_upper,
				  const std::string& _port_lower);
    public: ~AeroControllerNode();

    // private: void GoVelocityCallback(
    //     const geometry_msgs::Twist::ConstPtr& _msg);

    private: void JointTrajectoryCallback(
        const trajectory_msgs::JointTrajectory::ConstPtr& _msg);

    private: void JointStateCallback(const ros::TimerEvent& _event);

    private: void JointStateOnce();

    private: void WheelServoCallback(
        const std_msgs::Bool::ConstPtr& _msg);

    private: void WheelCommandCallback(
	const trajectory_msgs::JointTrajectory::ConstPtr& _msg);

    private: AeroUpperController upper_;

    private: AeroLowerController lower_;

    private: ros::NodeHandle nh_;

    // private: ros::Subscriber cmdvel_sub_;

    private: ros::Subscriber jointtraj_sub_;

    private: ros::Subscriber wheel_servo_sub_;

    private: ros::Subscriber wheel_sub_;

    private: ros::Publisher state_pub_;

    private: ros::Publisher stroke_state_pub_;

    private: ros::Timer timer_;

    private: boost::mutex mtx_;
    };

    typedef std::shared_ptr<AeroControllerNode> AeroControllerNodePtr;

  }
}

#endif
