#include "aero_move_base/AeroMoveBase.hh"

using namespace aero;
using namespace navigation;

//////////////////////////////////////////////////
/// @brief constructor
/// @param _nh ROS Node Handle
AeroMoveBase::AeroMoveBase(const ros::NodeHandle& _nh) :
  nh_(_nh),
  as_(nh_, "move_base/goal", false)
{
  this->Init();

  warm_up_time_ = 1.5;
  wait_for_servo_usec_ = 1000 * 500;

  goal_.wheel_dV.resize(num_of_wheels_);
  goal_.max_vel.resize(num_of_wheels_);
  states_.cur_vel.resize(num_of_wheels_);

  wheel_pub_ =
      nh_.advertise<trajectory_msgs::JointTrajectory>(
          "/aero_controller/wheel_command", 10);

  servo_pub_ =
      nh_.advertise<std_msgs::Bool>("/aero_controller/wheel_servo", 10);

  simple_goal_sub_ =
      nh_.subscribe("move_base_simple/goal",
                    10, &AeroMoveBase::SetSimpleGoal, this);

  timer_ =
      nh_.createTimer(ros::Duration(ros_rate_),
                      &AeroMoveBase::MoveBase, this);

  as_.registerGoalCallback(boost::bind(&AeroMoveBase::SetActionGoal, this));
  as_.registerPreemptCallback(boost::bind(&AeroMoveBase::CancelGoal, this));

  as_.start();
}

//////////////////////////////////////////////////
/// @brief destructor
AeroMoveBase::~AeroMoveBase()
{
}

//////////////////////////////////////////////////
void AeroMoveBase::MoveBase(const ros::TimerEvent& _event)
{
  if (!as_.isActive()) return;

  if (this->MoveBaseOnce())
  {
    geometry_msgs::PoseStamped feedback;
    feedback.pose.position.x = states_.moved_distance.x;
    feedback.pose.position.y = states_.moved_distance.y;
    feedback.pose.position.z = 0.0;
    feedback.pose.orientation.x = sin(0.5 * states_.moved_distance.theta);
    feedback.pose.orientation.y = 0.0;
    feedback.pose.orientation.z = 0.0;
    feedback.pose.orientation.w = cos(0.5 * states_.moved_distance.theta);
    feedback_.base_position = feedback;
    as_.publishFeedback(feedback_);
  }
  else if (states_.cur_time > goal_.run_time)
  {
    move_base_msgs::MoveBaseResult result;
    as_.setSucceeded(result);
  }
  else
  {
    move_base_msgs::MoveBaseResult result;
    as_.setAborted(result);
  }
}

//////////////////////////////////////////////////
bool AeroMoveBase::MoveBaseOnce()
{
  if (!states_.wheel_on) return false;

  if (states_.cur_time > goal_.run_time)
  {
    this->FinishMove();
    return false;
  }

  if (states_.cur_time <= goal_.warm_up_time)
  {
    ROS_INFO(" Accel");
    for (unsigned int i = 0; i < num_of_wheels_; ++i)
    {
      states_.cur_vel[i] += goal_.wheel_dV[i];
      if (fabs(states_.cur_vel[i]) > fabs(goal_.max_vel[i]))
      {
        states_.cur_vel[i] = goal_.max_vel[i];
      }
    }
  } 
  else if (states_.cur_time >= (goal_.run_time - goal_.warm_up_time))
  {
    ROS_INFO(" Break");
    for (unsigned int i = 0; i < goal_.wheel_dV.size(); ++i)
    {
      states_.cur_vel[i] -= goal_.wheel_dV[i];
      if (goal_.wheel_dV[i] > 0 && states_.cur_vel[i] < 0)
      {
        states_.cur_vel[i] = 0.0;
      }
      if (goal_.wheel_dV[i] < 0 && states_.cur_vel[i] > 0)
      {
        states_.cur_vel[i] = 0.0;
      }
    }
  }
  else
  {
    ROS_INFO(" Conti");
    for (unsigned int i = 0; i < num_of_wheels_; ++i)
    {
      states_.cur_vel[i] = goal_.max_vel[i];
    }
  }
  ROS_INFO("  vel: %f, %f, %f, %f",
	   states_.cur_vel[0],
	   states_.cur_vel[1],
	   states_.cur_vel[2],
	   states_.cur_vel[3]);

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = wheel_names_;
  msg.points.resize(1);
  msg.points[0].positions = states_.cur_vel;
  msg.points[0].time_from_start = ros::Duration(ros_rate_);
  wheel_pub_.publish(msg);

  states_.cur_time += ros_rate_;

  pose moved_fraction = this->dX(states_.cur_vel, ros_rate_);

  states_.moved_distance.x += moved_fraction.x;
  states_.moved_distance.y += moved_fraction.y;
  states_.moved_distance.theta += moved_fraction.theta;

  return true;
};

//////////////////////////////////////////////////
/// @brief set goal position and servo wheels
///
/// This function will call Translate() to determin wheel command.
void AeroMoveBase::SetGoal(float _x, float _y, float _theta)
{
  wheels wheel_data = this->Translate(_x, _y, _theta);

  // set goals
  goal_.run_time = wheel_data.time;
  ROS_INFO(" wheel_data.time = %f", wheel_data.time);
  ROS_INFO(" wheel_data.velocities[0] = %f", wheel_data.velocities[0]);

  goal_.warm_up_time = warm_up_time_;
  // if move within 2 * warm_up_time seconds
  if (goal_.run_time < (2.0 * warm_up_time_))
    goal_.warm_up_time =
        static_cast<float>(
            static_cast<int>(goal_.run_time * 0.5 / ros_rate_)
            * ros_rate_);
  // steps means how many steps MoveBaseOnce will run in warm up
  int steps = static_cast<int>(goal_.warm_up_time / ros_rate_);
  // if steps == 0, set 1 step and no warm up time
  if (steps == 0) steps = 1;
  // or add warm_up_time sec delay because of warm up
  else goal_.run_time += goal_.warm_up_time;

  ROS_INFO(" goal_.run_time = %f", goal_.run_time);
  ROS_INFO(" goal_.warm_up_time = %f", goal_.warm_up_time);
  ROS_INFO(" steps = %d", steps);
  for (unsigned int i = 0; i < wheel_data.velocities.size(); ++i)
  {
    goal_.wheel_dV[i] = wheel_data.velocities[i] / steps;
    goal_.max_vel[i] = wheel_data.velocities[i];
  }

  // set states
  std::fill(states_.cur_vel.begin(), states_.cur_vel.end(), 0.0);
  states_.cur_time = 0.0;
  states_.moved_distance = {0.0, 0.0, 0.0};

  // start wheels
  ROS_INFO(" Wheel Servo On");
  std_msgs::Bool dat;
  dat.data = true;
  servo_pub_.publish(dat);

  usleep(wait_for_servo_usec_);  // sleep until servo_on finished

  states_.wheel_on = true;
}

//////////////////////////////////////////////////
/// @brief set goal position via action,
/// this function is registered as GoalCallback of action server.
///
/// This function will call SetGoal().
void AeroMoveBase::SetActionGoal()
{
  geometry_msgs::PoseStamped goal = as_.acceptNewGoal()->target_pose;
  float theta = acos(goal.pose.orientation.w) * 2;
  int sgn = 0;
  if (goal.pose.orientation.x < 0) sgn = -1;
  else if (goal.pose.orientation.x > 0) sgn = 1;
  this->SetGoal(goal.pose.position.x, goal.pose.position.y, sgn * theta);
}

//////////////////////////////////////////////////
/// @brief cancel move command and base will stop immediately,
/// this function is registered as PreemptCallback of action server
void AeroMoveBase::CancelGoal()
{
  this->FinishMove();
  as_.setPreempted();
}

//////////////////////////////////////////////////
void AeroMoveBase::SetSimpleGoal(
    const geometry_msgs::PoseStamped::ConstPtr& _msg)
{
  float theta = acos(_msg->pose.orientation.w) * 2;
  int sgn = 0;
  if (_msg->pose.orientation.x < 0) sgn = -1;
  else if (_msg->pose.orientation.x > 0) sgn = 1;

  ROS_INFO("x = %f, y = %f", _msg->pose.position.x, _msg->pose.position.y);
  ROS_INFO("theta = %f, sgn = %d", theta, sgn);
  this->SetGoal(_msg->pose.position.x, _msg->pose.position.y, sgn * theta);

  while (this->MoveBaseOnce())
    usleep(ros_rate_ * 1000 * 1000);
}

//////////////////////////////////////////////////
/// @brief stop and servo off all wheels
void AeroMoveBase::FinishMove()
{
  // stop wheels
  // send zero for all wheels
  std::vector<double> zeros(num_of_wheels_, 0.0);

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = wheel_names_;
  msg.points.resize(1);
  msg.points[0].positions = zeros;
  msg.points[0].time_from_start = ros::Duration(ros_rate_);
  wheel_pub_.publish(msg);

  usleep(wait_for_servo_usec_);  // sleep until wheel command finished

  // wheel_off
  ROS_INFO(" Wheel Servo Off");
  std_msgs::Bool dat;
  dat.data = false;
  servo_pub_.publish(dat);

  states_.wheel_on = false;

  // reset goals and states in next goal call
  // does not reset here for future references
}
