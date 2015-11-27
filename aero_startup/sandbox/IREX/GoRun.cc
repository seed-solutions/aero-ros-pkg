#include <ros/ros.h>
#include <chrono>
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
#include <aero_startup/PointXYZHSI.h>

/*
  @define srv 1
  string object
  float32 x_cap
  float32 y_cap
  float32 z_cap
  float32 x
  float32 y
  float32 z
  int8 h_cap
  uint8 s_cap
  uint8 i_cap
  int8 h
  uint8 s
  uint8 i
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

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;

static const int GIVE_UP_TIME = 30000; // ms

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
  // set precise to false
  aero_startup::PointXYZHSI srv;
  bool prior_setting;
  srv.request.x_cap = req.x_cap;
  srv.request.y_cap = req.y_cap;
  srv.request.z_cap = req.z_cap;
  srv.request.x = req.x;
  srv.request.y = req.y;
  srv.request.z = req.z;
  srv.request.h_cap = req.h_cap;
  srv.request.s_cap = req.s_cap;
  srv.request.i_cap = req.i_cap;
  srv.request.h = req.h;
  srv.request.s = req.s;
  srv.request.i = req.i;
  srv.request.precise = false;
  if (client.call(srv))
  {
    prior_setting = srv.response.prior_setting;
  }
  else
  {
    ROS_ERROR("unexpected call fail to service when starting");
    res.status = -3;
    return true;
  }

  float diff_to_object_x;
  float diff_to_object_y;
  static tf::TransformListener tl;
  tf::StampedTransform base_to_object;
  ros::Time now = ros::Time::now();
  tl.waitForTransform("leg_base_link", req.object, now, ros::Duration(2.0));

  // find current distance to target object
  try
  {
    tl.lookupTransform("leg_base_link", req.object, now, base_to_object);
    diff_to_object_x = base_to_object.getOrigin().x();
    diff_to_object_y = base_to_object.getOrigin().y();
  }
  catch (std::exception e)
  {
    ROS_ERROR("failed tf listen during start of call");
    res.status = -3;
    return true;
  }

  // move robot forward
  GoPos(req.go_x, req.go_y, 0);
  float initial_diff = diff_to_object_x;

  ROS_WARN("Going for target : %f", initial_diff);

  Clock::time_point start = Clock::now();

  // stop when the robot is close enough to target object
  while ((diff_to_object_x > req.until_x) || (diff_to_object_y > req.until_y))
  {
    now = ros::Time::now();
    tl.waitForTransform("leg_base_link", req.object, now, ros::Duration(2.0));

    try
    {
      tl.lookupTransform("leg_base_link", req.object, now, base_to_object);
      diff_to_object_x = base_to_object.getOrigin().x();
      diff_to_object_y = base_to_object.getOrigin().y();
    }
    catch (std::exception e)
    {
      ROS_ERROR("failed tf listen during run");
      ac->cancelGoal();
      res.status = -2;
      return true;
    }

    // update search area
    srv.request.x_cap = req.x_cap;
    srv.request.y_cap = req.y_cap;
    srv.request.z_cap = req.z_cap;
    srv.request.x = req.x;
    srv.request.y = req.y;
    srv.request.z = req.z - (initial_diff - diff_to_object_x);
    srv.request.h_cap = req.h_cap;
    srv.request.s_cap = req.s_cap;
    srv.request.i_cap = req.i_cap;
    srv.request.h = req.h;
    srv.request.s = req.s;
    srv.request.i = req.i;
    srv.request.precise = false;
    if (client.call(srv))
    {
      if (srv.response.status <= 0)
      {
	ROS_ERROR("object lost");
	ac->cancelGoal();
	res.status = -2;
	return true;
      }
    }
    else
    {
      ROS_ERROR("unexpected call fail to service");
      ac->cancelGoal();
      res.status = -2;
      return true;
    }

    ROS_WARN("running");
    usleep(500 * 1000);

    Clock::time_point time_now = Clock::now();
    if (std::chrono::duration_cast<milliseconds>(time_now - start).count()
	> GIVE_UP_TIME)
    {
      ROS_ERROR("too long of a run!");
      ac->cancelGoal();
      res.status = -4;
      return true;
    }
  }

  ac->cancelGoal();
  res.status = 1;

  ROS_WARN("finished");

  Clock::time_point end = Clock::now();
  res.time =
      std::chrono::duration_cast<milliseconds>(end - start).count() / 1000.0;

  // set settings back to original
  srv.request.x_cap = req.x_cap;
  srv.request.y_cap = req.y_cap;
  srv.request.z_cap = req.z_cap;
  srv.request.x = req.x;
  srv.request.y = req.y;
  srv.request.z = req.z - (initial_diff - diff_to_object_x);
  srv.request.h_cap = req.h_cap;
  srv.request.s_cap = req.s_cap;
  srv.request.i_cap = req.i_cap;
  srv.request.h = req.h;
  srv.request.s = req.s;
  srv.request.i = req.i;
  srv.request.precise = prior_setting;
  if (!client.call(srv))
  {
    ROS_ERROR("failed to set settings back to prior");
    res.status = -1;
  }

  return true;
};

//////////////////////////////////////////////////
bool GoForTime(aero_startup::GoTime::Request  &req,
	       aero_startup::GoTime::Response &res)
{
  GoPos(req.go_x, req.go_y, req.go_theta);

  usleep(req.time * 1000 * 1000);

  ac->cancelGoal();
  res.status = 1;

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

  client = nh.serviceClient<aero_startup::PointXYZHSI>(
      "/extract_object_features/perception_area");

  ros::spin();

  return 0;
}
