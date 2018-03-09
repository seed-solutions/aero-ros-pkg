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

#ifndef __ROBOT_INTERFACE__
#define __ROBOT_INTERFACE__

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

namespace robot_interface
{

typedef std::vector<double > angle_vector;
typedef std::vector<double > time_vector;
typedef std::vector< std::vector<double > > angle_vector_sequence;
typedef std::map< std::string, double > joint_angle_map;
typedef std::map< int, double > index_angle_map;

class TrajectoryBase
{
public:
  TrajectoryBase() {}
  TrajectoryBase(const std::vector< std::string > &_joint_list) : joint_list_(_joint_list), start_offset_(0.02)
  { }

  ~TrajectoryBase() {};
#if 0
  virtual bool convertToAngleVector(const index_angle_map &_imap,
                                    angle_vector &_av);
#endif
  virtual bool convertToAngleVector(const joint_angle_map &_jmap,
                                    angle_vector &_av);
  virtual bool convertToAngleVector(const std::vector < std::string> &_names,
                                    const std::vector< double >      &_positions,
                                    angle_vector &_av);
  virtual bool convertToMap(const angle_vector &_av, joint_angle_map &_jmap);

  virtual bool sendAngles(const joint_angle_map &_jmap,
                          const double _tm, const ros::Time &_start);
  virtual bool sendAngles(const std::vector < std::string> &_names,
                          const std::vector< double>       &_positions,
                          const double _tm, const ros::Time &_start);

  virtual void send_angle_vector(const angle_vector &_av, const double _tm);
  virtual void send_angle_vector(const angle_vector &_av, const double _tm, const ros::Time &_start) = 0;

  virtual void send_angle_vector_sequence(const angle_vector_sequence &_av_seq, const time_vector &_tm_seq);
  virtual void send_angle_vector_sequence(const angle_vector_sequence &_av_seq, const time_vector &_tm_seq, const ros::Time &_start) = 0;

  virtual void reference_vector(angle_vector &_ref);
  virtual void potentio_vector (angle_vector &_ref);

  virtual bool wait_interpolation(double _tm = 0.0) = 0;

#if 0
  virtual bool interpolatingp() = 0;
  virtual void stop_motion(double _stop_time = 0.0) = 0;
  virtual void cancel_angle_vector (const std::string &_name, bool _wait) = 0;
#endif

  virtual void getReferencePositions( joint_angle_map &_map) = 0;
  virtual void getActualPositions   ( joint_angle_map &_map) = 0;

  virtual const std::vector < std::string > &getJointNames() { return joint_list_; }

  void setName(const std::string &_name) {
    name_ = _name;
  }

  const std::string &getName() {
    return name_;
  }

protected:
  std::vector< std::string > joint_list_;
  std::string name_;
  double start_offset_;
};

class TrajectoryClient : public actionlib::SimpleActionClient < control_msgs::FollowJointTrajectoryAction >, public TrajectoryBase
{

public:
  typedef boost::shared_ptr< TrajectoryClient> Ptr;
  typedef actionlib::SimpleActionClient < control_msgs::FollowJointTrajectoryAction > ClientBase;

public:
  TrajectoryClient(ros::NodeHandle &_nh,
                   const std::string &_act_name,
                   const std::string &_state_name,
                   const std::vector<std::string > &_jnames);
  ~TrajectoryClient();

  virtual void getReferencePositions( std::map < std::string, double> &_map);
  virtual void getActualPositions   ( std::map < std::string, double> &_map);

  virtual bool wait_interpolation(double _tm = 0.0)
  {
    if(!sending_goal_) {
      return true;
    }
    ros::Duration d(_tm);
    if( !(this->waitForResult(d)) ) {
      return false;
    }
    return true;
  }

  using TrajectoryBase::send_angle_vector;
  virtual void send_angle_vector(const angle_vector &_av, const double _tm, const ros::Time &_start);

  using TrajectoryBase::send_angle_vector_sequence;
  virtual void send_angle_vector_sequence(const angle_vector_sequence &_av_seq, const time_vector &_tm_seq, const ros::Time &_start);

public:
  void doneCb(const actionlib::SimpleClientGoalState& state,
              const control_msgs::FollowJointTrajectoryResultConstPtr& result) {
    ROS_DEBUG("done: %s", name_.c_str());
  }
  void activeCb() {
    ROS_DEBUG("active: %s", name_.c_str());
  }
  void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& result) {
    ROS_DEBUG("feedback: %s", name_.c_str());
  }

private:
  //// callback
  void StateCallback_(const control_msgs::JointTrajectoryControllerState::ConstPtr & _msg);

  ros::Subscriber state_sub_;
  control_msgs::JointTrajectoryControllerState current_state_;

  ros::CallbackQueue state_queue_;
  boost::shared_ptr < ros::AsyncSpinner > state_spinner_;

  boost::mutex state_mtx_;
  bool sending_goal_;
  bool updated_state_;
};

class RobotInterface : public TrajectoryBase
{

private:
  typedef std::map <std::string,
                    TrajectoryClient::Ptr > controller_map;
public:
  typedef boost::shared_ptr< RobotInterface> Ptr;

public:
  RobotInterface(ros::NodeHandle &_nh);
  ~RobotInterface();

  virtual bool sendAngles(const joint_angle_map &_jmap,
                          const double _tm, const ros::Time &_start);
  virtual bool sendAngles(const std::vector < std::string> &_names,
                          const std::vector< double>       &_positions,
                          const double _tm, const ros::Time &_start);

  using TrajectoryBase::send_angle_vector;
  virtual void send_angle_vector(const angle_vector &_av, const double _tm, const std::string &_name);
  virtual void send_angle_vector(const angle_vector &_av, const double _tm, const std::vector< std::string> &_names);
  virtual void send_angle_vector(const angle_vector &_av, const double _tm, const ros::Time &_start);

  using TrajectoryBase::send_angle_vector_sequence;
  virtual void send_angle_vector_sequence(const angle_vector_sequence &_av_seq, const time_vector &_tm_seq, const std::string &_name);
  virtual void send_angle_vector_sequence(const angle_vector_sequence &_av_seq, const time_vector &_tm_seq, const std::vector< std::string> &_names);
  virtual void send_angle_vector_sequence(const angle_vector_sequence &_av_seq, const time_vector &_tm_seq, const ros::Time &_start);

  virtual void getReferencePositions( std::map < std::string, double> &_map);
  virtual void getActualPositions   ( std::map < std::string, double> &_map);

  using TrajectoryBase::wait_interpolation;
  virtual bool wait_interpolation(double _tm = 0.0);
  virtual bool wait_interpolation(const std::string &_name, double _tm = 0.0);
  virtual bool wait_interpolation(const std::vector < std::string> &_names, double _tm = 0.0);

#if 0
  bool interpolatingp ();
  bool interpolatingp (const std::string &_name);

  void stop_motion(double _stop_time = 0.0);
  void cancel_angle_vector (const std::string &_name, bool _wait);
#endif

  bool add_controller (const std::string &_key,
                       const std::string &_action_name,
                       const std::string &_state_name,
                       const std::vector< std::string> &_jnames,
                       bool _update_joint_list = true);
  bool add_controller (const std::string &_key,
                       const TrajectoryClient::Ptr &_p,
                       bool _update_joint_list = true);

  bool defineJointList(std::vector < std::string > &_jl);

  bool updateJointList();
  bool updateJointList(std::vector < std::string > &_controller_names_list);

private:
  //// callback
  void JointStateCallback_(const sensor_msgs::JointState::ConstPtr& _msg);
  void group2names_ (const std::string &_group, std::vector <std::string> &_names)
  {
    _names.resize(0);
    auto it = controller_group_.find(_group);
    if (it != controller_group_.end()) {
      _names = it->second;
    }
  }
  bool wait_interpolation_(const std::string &_name, double _tm = 0.0);

protected:
  controller_map controllers_;
  ros::NodeHandle local_nh_;

  ros::Subscriber joint_states_sub_;

  ros::Time current_stamp_;
  std::map < std::string, double> current_positions_;
  std::map < std::string, double> current_velocities_;
  std::map < std::string, double> current_effort_;

  boost::mutex states_mtx_;
  ros::CallbackQueue joint_states_queue_;
  boost::shared_ptr < ros::AsyncSpinner > joint_states_spinner_;

  std::map< std::string, std::vector<std::string > > controller_group_;
  bool updated_joint_state_;
};

}

#endif // __ROBOT_INTERFACE__
