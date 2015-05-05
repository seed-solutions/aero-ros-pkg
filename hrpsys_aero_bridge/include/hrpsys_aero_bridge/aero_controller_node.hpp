#ifndef HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_NODE_HPP_
#define HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_NODE_HPP_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>

#include "hrpsys_aero_bridge/constants.hpp"
#include "hrpsys_aero_bridge/aero_controller.hpp"

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

namespace aero_controller {

class AeroControllerNode {
 public:
  AeroControllerNode(const ros::NodeHandle& nh,
                     const ros::NodeHandle& pnh,
                     std::string port_upper, std::string port_lower);
  ~AeroControllerNode();

  void goVelocityCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void jointTrajectoryCallback(
      const trajectory_msgs::JointTrajectory::ConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  AeroController controller_;

  ros::Subscriber cmdvel_sub_;
  ros::Subscriber jointtraj_sub_;
};

}  // namespace

#endif  // HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_NODE_HPP_
