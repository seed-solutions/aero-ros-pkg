#ifndef AERO_NAVIGATION_AERO_MOVE_BASE_H_
#define AERO_NAVIGATION_AERO_MOVE_BASE_H_

#include <vector>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <geometry_msgs/PoseStamped.h>

namespace aero
{
  namespace navigation
  {

    struct wheels
    {
      std::vector<float> velocities;

      float time;
    };

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

    class AeroMoveBase
    {
    public: explicit AeroMoveBase(const ros::NodeHandle& _nh);

    public: ~AeroMoveBase();

    private: void Init();

    private: void MoveBase(const ros::TimerEvent& _event);

    private: bool MoveBaseOnce();

    private: wheels Translate(float _x, float _y, float _theta);

    private: pose dX(std::vector<double> _vels, float _dt);

    private: void SetSimpleGoal(
        const geometry_msgs::PoseStamped::ConstPtr& _msg);

    private: void SetActionGoal();

    private: void CancelGoal();

    private: void FinishMove();

    private: void SetGoal(float _x, float _y, float _theta);

    private: std::vector<std::string> wheel_names_;

    private: float ros_rate_;

    private: int num_of_wheels_;

    private: goal goal_;

    private: states states_;

    private: ros::NodeHandle nh_;

    private: actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;

    private: move_base_msgs::MoveBaseFeedback feedback_;

    private: ros::Publisher wheel_pub_;

    private: ros::Publisher servo_pub_;

    private: ros::Subscriber simple_goal_sub_;

    private: ros::Timer timer_;

    private: class AeroMoveBaseImpl;

    private: std::shared_ptr<AeroMoveBaseImpl> impl_;
    };

    typedef std::shared_ptr<AeroMoveBase> AeroMoveBasePtr;

  }
}

#endif
