#ifndef _AERO_NAVIGATION_RUNNER_H_
#define _AERO_NAVIGATION_RUNNER_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

namespace aero
{
  namespace navigation
  {

    class Runner
    {
    public: explicit Runner(ros::NodeHandle _nh);

    public: ~Runner();

    protected: void GoPos(float _x, float _y, float _theta);

    protected: ros::NodeHandle nh_;

    protected: actionlib::SimpleActionClient<
        move_base_msgs::MoveBaseAction> *ac_;
    };

  }
}

#endif
