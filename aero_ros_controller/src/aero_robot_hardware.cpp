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

#include "aero_robot_hardware.h"
#include <urdf/model.h>

#include <thread>

namespace aero_robot_hardware
{

bool AeroRobotHW::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)// add joint list
{
  std::string port_upper("/dev/aero_upper");
  std::string port_lower("/dev/aero_lower");

  // reading paramerters
  if (robot_hw_nh.hasParam("port_upper")) {
    robot_hw_nh.getParam("port_upper", port_upper);
  }
  if (robot_hw_nh.hasParam("port_lower")) {
    robot_hw_nh.getParam("port_lower", port_lower);
  }
  if (robot_hw_nh.hasParam("controller_rate")) {
    double rate;
    robot_hw_nh.getParam("controller_rate", rate);
    CONTROL_PERIOD_US_ = (1000*1000)/rate;
  } else {
    CONTROL_PERIOD_US_ = 50*1000; // 50ms
  }
  if (robot_hw_nh.hasParam("overwarp_scale")) {
    double scl;
    robot_hw_nh.getParam("overwarp_scale", scl);
    OVERWRAP_SCALE_    = scl;     //
  }  else {
    OVERWRAP_SCALE_    = 2.8;     //
  }

  // create controllersd
  controller_upper_.reset(new AeroUpperController(port_upper));
  controller_lower_.reset(new AeroLowerController(port_lower));

  // joint list
  number_of_angles_ =
    controller_upper_->get_number_of_angle_joints() +
    controller_lower_->get_number_of_angle_joints();

  joint_list_.resize(number_of_angles_);
  for(int i = 0; i < number_of_angles_; i++) {
    std::string name;
    if(controller_upper_->get_joint_name(i, name)) {
      joint_list_[i] = name;
    } else if (controller_lower_->get_joint_name(i, name)) {
      joint_list_[i] = name;
    } else {
      std::cerr << "[WARN] name of joint " << i << "can not find!" << std::endl;
    }
  }

  // stroke list
#if 0
  number_of_strokes_ =
    controller_upper_->get_number_of_strokes() +
    controller_lower_->get_number_of_strokes();
  stroke_list_.resize(0);
  for(int i = 0; i < controller_upper_->get_number_of_strokes(); i++) {
    std::string name = controller_upper_->get_stroke_joint_name(i);
    stroke_list_.push_back(name);
  }
  for(int i = 0; i < controller_lower_->get_number_of_strokes(); i++) {
    std::string name = controller_lower_->get_stroke_joint_name(i);
    stroke_list_.push_back(name);
  }
#endif
  prev_ref_positions_.resize(number_of_angles_);
  initialized_flag_ = false;

  std::string model_str;
  if (!root_nh.getParam("robot_description", model_str)) {
    ROS_ERROR("Failed to get model from robot_description");
    return false;
  }
  urdf::Model model;
  if (!model.initString(model_str)) {
    ROS_ERROR("Failed to parse robot_description");
    return false;
  }

  ROS_DEBUG("read %d joints", number_of_angles_);
  for (int i = 0; i < number_of_angles_; i++) {
    ROS_DEBUG("  %d: %s", i, joint_list_[i].c_str());
    if(!model.getJoint(joint_list_[i])) {
      ROS_ERROR("Joint %s does not exist in urdf model", joint_list_[i].c_str());
      return false;
    }
  }

  // joint_names_.resize(number_of_angles_);
  joint_types_.resize(number_of_angles_);
  joint_lower_limits_.resize(number_of_angles_);
  joint_upper_limits_.resize(number_of_angles_);
  joint_effort_limits_.resize(number_of_angles_);
  joint_control_methods_.resize(number_of_angles_);
  //pid_controllers_.resize(number_of_angles_);
  joint_position_.resize(number_of_angles_);
  joint_velocity_.resize(number_of_angles_);
  joint_effort_.resize(number_of_angles_);
  joint_effort_command_.resize(number_of_angles_);
  joint_position_command_.resize(number_of_angles_);
  joint_velocity_command_.resize(number_of_angles_);

  readPos(ros::Time::now(), ros::Duration(0.0), true); /// initial

  // Initialize values
  for(unsigned int j = 0; j < number_of_angles_; j++) {
    // Add data from transmission
    //joint_position_[j]         = 0.0; // initialize
    //joint_position_command_[j] = 0.0; //

    joint_velocity_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;
    joint_effort_[j]   = 0.0;  // N/m for continuous joints
    joint_effort_command_[j]   = 0.0;

    std::string jointname = joint_list_[j];
    // Create joint state interface for all joints
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        jointname, &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    joint_control_methods_[j] = POSITION;
    hardware_interface::JointHandle joint_handle =
      hardware_interface::JointHandle(js_interface_.getHandle(jointname),
                                      &joint_position_command_[j]);
    pj_interface_.registerHandle(joint_handle);

    joint_limits_interface::JointLimits limits;
    const bool urdf_limits_ok = joint_limits_interface::getJointLimits(model.getJoint(jointname), limits);
    if (!urdf_limits_ok) {
      ROS_WARN("urdf limits of joint %s is not defined", jointname.c_str());
    }
    // Register handle in joint limits interface
    joint_limits_interface::PositionJointSaturationHandle
      limits_handle(joint_handle, // We read the state and read/write the command
                    limits);       // Limits spec
    pj_sat_interface_.registerHandle(limits_handle);
  }
  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&pj_interface_);
  //registerInterface(&vj_interface_);
  //registerInterface(&ej_interface_);

  return true;
}

void AeroRobotHW::readPos(const ros::Time& time, const ros::Duration& period, bool update)
{
  /////
  ROS_DEBUG("read");

  mutex_lower_.lock();

  // TODO: thrading?? or making no wait
  if (update) {
#if 0 // NO_THREAD
    controller_upper_->update_position();
    controller_lower_->update_position();
#else
    std::thread t1([&](){
        controller_upper_->update_position();
      });
    std::thread t2([&](){
        controller_lower_->update_position();
      });
    t1.join();
    t2.join();
#endif
  }
  // get upper actual positions
  std::vector<int16_t> upper_act_strokes =
    controller_upper_->get_actual_stroke_vector();
  // get lower actual positions
  std::vector<int16_t> lower_act_strokes =
    controller_lower_->get_actual_stroke_vector();

  mutex_lower_.unlock();

  // whole body strokes
  std::vector<int16_t> act_strokes (AERO_DOF_UPPER + AERO_DOF_LOWER);
  if (upper_act_strokes.size() < AERO_DOF_UPPER) {
    for (size_t i = 0; i < AERO_DOF_UPPER; ++i) {
      act_strokes[i] = 0;
    }
  } else { // usually should enter else, enters if when port is not activated
    for (size_t i = 0; i < AERO_DOF_UPPER; ++i) {
      act_strokes[i] = upper_act_strokes[i];
    }
  }
  if ( lower_act_strokes.size() < AERO_DOF_LOWER ) {
    for (size_t i = 0; i < AERO_DOF_LOWER; ++i) {
      act_strokes[i + AERO_DOF_UPPER] = 0; //??
    }
  } else { // usually should enter else, enters if when port is not activated
    for (size_t i = 0; i < AERO_DOF_LOWER; ++i) {
      act_strokes[i + AERO_DOF_UPPER] = lower_act_strokes[i];
    }
  }
  // whole body positions from strokes
  std::vector<double> act_positions;
  act_positions.resize(number_of_angles_);
  common::Stroke2Angle(act_positions, act_strokes);

  // DEBUG
  // act_strokes
  // act_positions
  //

  double tm = period.toSec();
  for(unsigned int j=0; j < number_of_angles_; j++) {
    float position = act_positions[j];
    float velocity = 0.0;

    if (joint_types_[j] == PRISMATIC) {
      joint_position_[j] = position;
    } else {
      //joint_position_[j] += angles::shortest_angular_distance(joint_position_[j], position);
      joint_position_[j] = position;
    }
    joint_velocity_[j] = velocity; // read velocity from HW
    joint_effort_[j]   = 0;        // read effort   from HW
  }

  if (!initialized_flag_) {
    for(unsigned int j = 0; j < number_of_angles_; j++) {
      joint_position_command_[j] = prev_ref_positions_[j] = joint_position_[j];
      ROS_DEBUG("%d: %s - %f", j, joint_list_[j].c_str(), joint_position_command_[j]);
    }
    initialized_flag_ = true;
  }
}

void AeroRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
  readPos(time, period, true);
  return;
}

void AeroRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
  ROS_DEBUG("write");

  pj_sat_interface_.enforceLimits(period);
  //pj_limits_interface_.enforceLimits(period);
  //vj_sat_interface_.enforceLimits(period);
  //vj_limits_interface_.enforceLimits(period);
  //ej_sat_interface_.enforceLimits(period);
  //ej_limits_interface_.enforceLimits(period);

  ////// convert poitions to strokes and write strokes
  std::vector<double > ref_positions(number_of_angles_);
  //std::fill(ref_positions.begin(), ref_positions.end(), 0.0);
  for(unsigned int j=0; j < number_of_angles_; j++) {
    switch (joint_control_methods_[j]) {
    case POSITION:
      {
        ref_positions[j] = joint_position_command_[j];
      }
      break;
    case VELOCITY:
      {
      }
      break;
    case EFFORT:
      {
      }
      break;
    case POSITION_PID:
      {
      }
      break;
    case VELOCITY_PID:
      {
      }
      break;
    } // switch
  } // for

  std::vector<bool > mask_positions(number_of_angles_);
  std::fill(mask_positions.begin(), mask_positions.end(), true); // send if true

  for(int i = 0; i < number_of_angles_; i++) {
    double tmp = ref_positions[i];
    if (tmp == prev_ref_positions_[i]) {
      mask_positions[i] = false;
    }
    prev_ref_positions_[i] = tmp;
  }

  std::vector<int16_t> ref_strokes(AERO_DOF);
  common::Angle2Stroke(ref_strokes, ref_positions);
  std::vector<int16_t> snt_strokes(ref_strokes);
  common::UnusedAngle2Stroke(snt_strokes, mask_positions);

  // split strokes into upper and lower
  std::vector<int16_t> upper_strokes(snt_strokes.begin(), snt_strokes.begin() + AERO_DOF_UPPER);
  std::vector<int16_t> lower_strokes(snt_strokes.begin() + AERO_DOF_UPPER, snt_strokes.end());

  uint16_t time_csec = static_cast<uint16_t>((OVERWRAP_SCALE_ * CONTROL_PERIOD_US_)/(1000*10));
  mutex_lower_.lock();
  {
#if 0 // NO_THREAD
    controller_upper_->set_position(upper_strokes, time_csec);
    controller_lower_->set_position(lower_strokes, time_csec);
#else
    std::thread t1([&](){
        controller_upper_->set_position(upper_strokes, time_csec);
      });
    std::thread t2([&](){
        controller_lower_->set_position(lower_strokes, time_csec);
      });
    t1.join();
    t2.join();
    usleep( 1000 * 2 ); // why needed?
  }
#endif
  mutex_lower_.unlock();

  // read
  //readPos(time, period, false);
#if 0
  for(unsigned int j=0; j < number_of_angles_; j++) {
    switch (joint_control_methods_[j]) {
    case POSITION:
      {
        // write position to hw
        shm_->ref_angle[j] = joint_position_command_[j];
      }
      break;
    case VELOCITY:
      {
      }
      break;
    case EFFORT:
      {
      }
      break;
    case POSITION_PID:
      {
      }
      break;
    case VELOCITY_PID:
      {
      }
      break;
    } // switch
  } // for
#endif
}

void AeroRobotHW::writeWheel(const std::vector< std::string> &_names, const std::vector<int16_t> &_vel, double _tm_sec) {
  ROS_DEBUG("wheel %d %d %d %d",
            _vel[0], _vel[1], _vel[2], _vel[3]);

  mutex_lower_.lock();
  std::vector<int32_t> joint_to_wheel_indices(AERO_DOF_WHEEL);
  for (size_t i = 0; i < _names.size(); ++i) {
    std::string joint_name = _names[i];
    joint_to_wheel_indices[i] =
      controller_lower_->get_wheel_id(joint_name);
  }
  std::vector<int16_t> wheel_vector;
  std::vector<int16_t>& ref_vector =
    controller_lower_->get_reference_wheel_vector();
  wheel_vector.assign(ref_vector.begin(), ref_vector.end());
  for (size_t j = 0; j < _vel.size(); ++j) {
    if (joint_to_wheel_indices[j] >= 0) {
      wheel_vector[static_cast<size_t>(joint_to_wheel_indices[j])] = _vel[j];
    }
  }
  uint16_t time_csec = static_cast<uint16_t>(_tm_sec * 100.0);
  controller_lower_->set_wheel_velocity(wheel_vector, time_csec);
  mutex_lower_.unlock();
}

void AeroRobotHW::startWheelServo() {
  ROS_DEBUG("servo on");

  mutex_lower_.lock();
  controller_lower_->wheel_on();
  mutex_lower_.unlock();
}

void AeroRobotHW::stopWheelServo() {
  ROS_DEBUG("servo off");

  mutex_lower_.lock();
  controller_lower_->wheel_only_off();
  mutex_lower_.unlock();
}

}

// PLUGINLIB_EXPORT_CLASS(universal_realtime_controller::UniversealRobotHWShm, gazebo_ros_control::RobotHWSim)
