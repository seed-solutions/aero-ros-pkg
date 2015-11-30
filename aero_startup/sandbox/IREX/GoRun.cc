#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <aero_startup/ObjectGoXYZHSI.h>
#include <aero_startup/GoTime.h>
#include <aero_startup/AutoTrackReconfigure.h>
#include <aero_startup/HeaderRequest.h>

/*
  @define srv 1
  string object
  float32 go_x
  float32 go_y
  float32 until_x
  float32 until_y
  ---
  int8 status
  float32 time
*/

/*
  @define srv 2
  float32 go_x
  float32 go_y
  float32 go_theta
  float32 time
  ---
  int8 status
*/

namespace aero
{
  namespace status
  {
    const static int success = 1;
    const static int warning = -1;
    const static int aborted = -2;
    const static int fatal = -4;
  }
};

static const int GIVE_UP_TIME = 20000; // ms

actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
ros::ServiceClient client;

//////////////////////////////////////////////////
void GoPos(float _x, float _y, float _theta)
{
  geometry_msgs::Point p;
  p.x = _x;
  p.y = _y;
  p.z = 0;

  geometry_msgs::Quaternion q;
  q.x = cos(_theta*M_PI/180);
  q.y = 0;
  q.z = 0;
  q.w = sin(_theta*M_PI/180);

  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;

  geometry_msgs::PoseStamped pstamp;
  pstamp.pose = pose;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = pstamp;

  ac->sendGoal(goal);
};

//////////////////////////////////////////////////
bool GoForTarget(aero_startup::ObjectGoXYZHSI::Request  &req,
		 aero_startup::ObjectGoXYZHSI::Response &res)
{
  // Note : target already in required area should not be called
  // Check for target requirements should be done prior to service call

  // move robot
  GoPos(req.go_x, req.go_y, 0);

  ROS_WARN("Going for target");

  // call dynamic_reconfigure
  aero_startup::AutoTrackReconfigure srv;
  srv.request.end_condition_x = req.until_x;
  srv.request.end_condition_y = req.until_y;
  srv.request.time_out = GIVE_UP_TIME;
  srv.request.precise = false;
  srv.request.precise_default = true; // set precise back to original value

  if (client.call(srv))
  {
    if (srv.response.status < 0)
    {
      ROS_ERROR("failed go for target");
      ac->cancelGoal();
      res.status = srv.response.status;
      return true;
    }
  }
  else
  {
    ROS_ERROR("unexpected call fail to service");
    ac->cancelGoal();
    res.status = aero::status::fatal;
    return true;
  }

  ac->cancelGoal();
  res.status = aero::status::success;
  res.time = srv.response.complete_time;

  ROS_WARN("finished");

  return true;
};

//////////////////////////////////////////////////
bool GoForTime(aero_startup::GoTime::Request  &req,
	       aero_startup::GoTime::Response &res)
{
  GoPos(req.go_x, req.go_y, req.go_theta);

  usleep(req.time * 1000 * 1000);

  ac->cancelGoal();
  res.status = aero::status::success;

  return true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_run");
  ros::NodeHandle nh;

  ac = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
      "/move_base/goal", true);

  ros::ServiceServer service_obj =
      nh.advertiseService("/go_run/target_object", GoForTarget);
  ros::ServiceServer service_tm =
      nh.advertiseService("/go_run/target_time", GoForTime);

  client = nh.serviceClient<aero_startup::AutoTrackReconfigure>(
      "/extract_object_features/dynamic_goal");

  ros::spin();

  return 0;
}
