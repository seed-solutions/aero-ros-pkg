#include "aero_navigation/navigation_local/class/RunByTime.hh"

using namespace aero;
using namespace navigation;

//////////////////////////////////////////////////
RunByTime::RunByTime(ros::NodeHandle _nh) : Runner(_nh)
{
  goal_service_ =
      nh_.advertiseService("/run_by_time/target_time",
			   &RunByTime::GoForTime, this);
}

//////////////////////////////////////////////////
RunByTime::~RunByTime()
{
}

//////////////////////////////////////////////////
bool RunByTime::GoForTime(aero_startup::PointGoTime::Request  &_req,
			  aero_startup::PointGoTime::Response &_res)
{
  GoPos(_req.go_x, _req.go_y, _req.go_theta);
  usleep(_req.time * 1000 * 1000);
  ac_->cancelGoal();
  _res.status = aero::status::success;
  return true;
}
