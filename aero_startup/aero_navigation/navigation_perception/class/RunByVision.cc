#include "aero_navigation/navigation_perception/class/RunByVision.hh"

using namespace aero;
using namespace navigation;

//////////////////////////////////////////////////
RunByVision::RunByVision(ros::NodeHandle _nh) : Runner(_nh)
{
  goal_service_ =
      nh_.advertiseService("/run_by_vision/target_object",
			   &RunByVision::GoForTarget, this);
  object_subscriber_ = nh_.subscribe("/object/pose", 100,
				     &RunByVision::Subscribe, this);
  recognition_mode_ = nh_.serviceClient<aero_startup::ProcessSleep>(
      "/plane_detection/sleep");
  tracking_mode_ = nh_.serviceClient<aero_startup::ProcessSleep>(
      "/object_tracker/sleep");
}

//////////////////////////////////////////////////
RunByVision::~RunByVision()
{
}

//////////////////////////////////////////////////
void RunByVision::Subscribe(const geometry_msgs::Pose::ConstPtr& _object)
{
  object_.x = _object->position.x;
  object_.y = _object->position.y;
  object_.z = _object->position.z;
}

//////////////////////////////////////////////////
bool RunByVision::GoForTarget(aero_startup::ObjectGoXYZHSI::Request  &_req,
			      aero_startup::ObjectGoXYZHSI::Response &_res)
{
  // RunByVision expects that target object is identified prior to call

  ros::spinOnce();

  if (fabs(object_.x) < 0.001 &&
      fabs(object_.y) < 0.001 &&
      fabs(object_.z) < 0.001) // object on base is impossible
  {
    _res.status = aero::status::aborted;
    return true;
  }

  // set recognition to tracking mode
  aero_startup::ProcessSleep srv;
  srv.request.sleep = 1;
  if (!recognition_mode_.call(srv))
  {
    ROS_WARN("call to recognition mode failed");
    _res.status = aero::status::aborted;
    return true;
  }
  srv.request.sleep = 0;
  srv.request.message = std::to_string(object_.x) + " " +
			std::to_string(object_.y) + " " +
			std::to_string(object_.z) + "world";
  if (!tracking_mode_.call(srv))
  {
    ROS_WARN("call to tracking mode failed");
    _res.status = aero::status::aborted;
    return true;
  }

  // handle y conditions
  std::function<bool(float)> y_condition;
  if (fabs(_req.until_y) > 999) // impossible goal, reject
  {
    y_condition = [=](float ref){ return false; };
  }
  else
  {
    if (_req.until_y >= 0)
      if (object_.y > _req.until_y)
	y_condition = [=](float ref)
	{
	  if (ref > _req.until_y) // continue condition
	    return true;
	  return false; // escapes at false
	};
      else if (object_.y < 0)
	y_condition = [=](float ref)
	{
	  if (ref < 0.0) // continue condition
	    return true;
	  return false; // escapes at false
	};
      else
	y_condition = [=](float ref){ return false; };
    else
      if (object_.y < _req.until_y)
	y_condition = [=](float ref)
	{
	  if (ref < _req.until_y) // continue condition
	    return true;
	  return false; // escapes at false
	};
      else if (object_.y > 0)
	y_condition = [=](float ref)
	{
	  if (ref > 0) // continue condition
	    return true;
	  return false; // escapes at false
	};
      else
	y_condition = [=](float ref){ return false; };
  }

  auto start = aero::time::now();
  bool not_run_yet = true;

  while ((object_.x > _req.until_x) ||
	 y_condition(object_.y))
  {
    // move robot
    if (not_run_yet)
    {
      GoPos(_req.go_x, _req.go_y, 0);
      not_run_yet = false;
    }

    ros::spinOnce();

    if (fabs(object_.x) < 0.001 &&
	fabs(object_.y) < 0.001 &&
	fabs(object_.z) < 0.001) // failed object detection
    {
      ac_->cancelGoal();
      _res.status = aero::status::aborted;
      return true;
    }

    auto time_now = aero::time::now();
    if (aero::time::ms(time_now - start) > _req.time_out_ms) // time_out
    {
      ac_->cancelGoal();
      _res.status = aero::status::fatal;
      return true;
    }
  }

  ac_->cancelGoal();
  auto end = aero::time::now();
  _res.time = aero::time::ms(end - start) / 1000.0;

  // set recognition back to recognition mode
  aero_startup::ProcessSleep vrs;
  vrs.request.sleep = 1;
  if (!tracking_mode_.call(vrs))
  {
    ROS_WARN("call to tracking mode failed");
    _res.status = aero::status::warning;
    return true;
  }
  vrs.request.sleep = 0;
  if (!recognition_mode_.call(vrs))
  {
    ROS_WARN("call to recognition mode failed");
    _res.status = aero::status::warning;
    return true;
  }

  _res.status = aero::status::success;
  return true;
}
