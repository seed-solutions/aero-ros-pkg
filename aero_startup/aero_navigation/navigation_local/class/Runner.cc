#include "aero_navigation/navigation_local/class/Runner.hh"

using namespace aero;
using namespace navigation;

//////////////////////////////////////////////////
Runner::Runner(ros::NodeHandle _nh) : nh_(_nh)
{
  ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(
       "/move_base/goal", true);
}

//////////////////////////////////////////////////
Runner::~Runner()
{
}

//////////////////////////////////////////////////
void Runner::GoPos(float _x, float _y, float _theta)
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

  ac_->sendGoal(goal);
}
