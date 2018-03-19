/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Yohei Kakiuchi (JSK lab.)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 Author: Yohei Kakiuchi
*/

#ifndef _AERO_ROBOT_HW_H_
#define _AERO_ROBOT_HW_H_

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>

// URDF
#include <urdf/model.h>

// AERO
#include "aero_hardware_interface/Constants.hh"
#include "aero_hardware_interface/AeroControllers.hh"

#include "aero_hardware_interface/AngleJointNames.hh"
#include "aero_hardware_interface/Stroke2Angle.hh"
#include "aero_hardware_interface/Angle2Stroke.hh"
#include "aero_hardware_interface/UnusedAngle2Stroke.hh"

#include <mutex>

using namespace aero;
using namespace controller;
using namespace common;

namespace aero_robot_hardware
{

class AeroRobotHW : public hardware_interface::RobotHW
{
public:
  AeroRobotHW() { }

  virtual ~AeroRobotHW() {}

  /** \brief The init function is called to initialize the RobotHW from a
   * non-realtime thread.
   *
   * \param root_nh A NodeHandle in the root of the caller namespace.
   *
   * \param robot_hw_nh A NodeHandle in the namespace from which the RobotHW
   * should read its configuration.
   *
   * \returns True if initialization was successful
   */
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh);


  /** \name Hardware Interface Switching
   *\{*/
  /**
   * Check (in non-realtime) if given controllers could be started and stopped from the current state of the RobotHW
   * with regard to necessary hardware interface switches and prepare the switching. Start and stop list are disjoint.
   * This handles the check and preparation, the actual switch is commited in doSwitch()
   */
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                             const std::list<hardware_interface::ControllerInfo>& stop_list) {
    // message
    return true;
  }

  /**
   * Perform (in realtime) all necessary hardware interface switches in order to start and stop the given controllers.
   * Start and stop list are disjoint. The feasability was checked in prepareSwitch() beforehand.
   */
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) {
    // message
  }

  /**
   * Reads data from the robot HW
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref read
   */
  virtual void read(const ros::Time& time, const ros::Duration& period);

  /**
   * Writes data to the robot HW
   *
   * \param time The current time
   * \param period The time passed since the last call to \ref write
   */
  virtual void write(const ros::Time& time, const ros::Duration& period);


  ///
  void readPos(const ros::Time& time, const ros::Duration& period, bool update);
  void writeWheel(const std::vector< std::string> &_names, const std::vector<int16_t> &_vel, double _tm_sec);
  void startWheelServo();
  void stopWheelServo();

  void handScript(uint16_t _sendnum, uint16_t _script) {
    mutex_upper_.lock();
    controller_upper_->Hand_Script(_sendnum, _script);
    mutex_upper_.unlock();
  }
  void setMaxSingleCurrent(uint16_t _sendnum, uint16_t _power) {
    mutex_upper_.lock();
    controller_upper_->set_max_single_current(_sendnum, _power);
    mutex_upper_.unlock();
  }

  double getPeriod() { return ((double)CONTROL_PERIOD_US_) / (1000 * 1000); }
  double getOverWrapScale() { return OVERWRAP_SCALE_; }

protected:
  // Methods used to control a joint.
  enum ControlMethod {EFFORT, POSITION, POSITION_PID, VELOCITY, VELOCITY_PID};
  enum JointType {NONE, PRISMATIC, ROTATIONAL, CONTINUOUS, FIXED};

  unsigned int number_of_angles_;

  hardware_interface::JointStateInterface    js_interface_;
  hardware_interface::PositionJointInterface pj_interface_;
  //hardware_interface::VelocityJointInterface vj_interface_;
  //hardware_interface::EffortJointInterface   ej_interface_;

  joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  //joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
  //joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
  //joint_limits_interface::VelocityJointSaturationInterface vj_sat_interface_;
  //joint_limits_interface::VelocityJointSoftLimitsInterface vj_limits_interface_;
  //joint_limits_interface::EffortJointSaturationInterface   ej_sat_interface_;
  //joint_limits_interface::EffortJointSoftLimitsInterface   ej_limits_interface_;

  std::vector<std::string> joint_list_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_effort_limits_;
  std::vector<JointType>     joint_types_;
  std::vector<ControlMethod> joint_control_methods_;
  // std::vector<control_toolbox::Pid> pid_controllers_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;

  std::vector<double> prev_ref_positions_;

  boost::shared_ptr<AeroUpperController > controller_upper_;
  boost::shared_ptr<AeroLowerController > controller_lower_;

  bool initialized_flag_;

  int   CONTROL_PERIOD_US_;
  float OVERWRAP_SCALE_;
  int   BASE_COMMAND_PERIOD_MS_;

  std::mutex mutex_lower_;
  std::mutex mutex_upper_;
};

typedef boost::shared_ptr<AeroRobotHW> AeroRobotHWPtr;
}

#endif // #ifndef _UNIVERSAL_ROBOT_HW_SHM_H_
