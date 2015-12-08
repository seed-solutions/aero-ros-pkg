#include "aero_navigation/navigation_perception/class/RunByVision.hh"

using namespace aero;
using namespace navigation;

//////////////////////////////////////////////////
RunByVision::RunByVision(ros::NodeHandle _nh,
			 perception::PointCloudSensorPtr _sensor,
			 perception::ObjectFeaturesPtr _object) : Runner(_nh)
{
  sensor_ = _sensor;
  object_ = _object;
  goal_service_ =
      nh_.advertiseService("/run_by_vision/target_object",
			   &RunByVision::GoForTarget, this);
}

//////////////////////////////////////////////////
RunByVision::~RunByVision()
{
}

//////////////////////////////////////////////////
bool RunByVision::GoForTarget(aero_startup::ObjectGoXYZHSI::Request  &_req,
			      aero_startup::ObjectGoXYZHSI::Response &_res)
{
  // Note : target already in required area should not be called
  // Check for target requirements should be done prior to service call

  // move robot
  GoPos(_req.go_x, _req.go_y, 0);

  ros::spinOnce();
  object_->ExtractObjectFeatures(
      sensor_->GetCenter(), sensor_->GetVertices(), sensor_->GetCloud());
  int status = object_->GetStatus();
  tf::StampedTransform base_to_eye = object_->GetBaseToEye();
  Eigen::Vector3f target_center_camera = object_->GetTargetCenterCamera();

  if (status <= aero::status::aborted) // failed object detection
  {
    _res.status = aero::status::aborted;
    return true;
  }

  Eigen::Quaternionf base_to_eye_q =
      Eigen::Quaternionf(base_to_eye.getRotation().w(),
                         base_to_eye.getRotation().x(),
                         base_to_eye.getRotation().y(),
                         base_to_eye.getRotation().z());;

  Eigen::Vector3f diff_to_object =
      Eigen::Vector3f(base_to_eye.getOrigin().x(),
                      base_to_eye.getOrigin().y(),
                      base_to_eye.getOrigin().z()) +
      base_to_eye_q * target_center_camera;

  sensor_->SetSpaceMax({target_center_camera[0], target_center_camera[1],
	target_center_camera[2] + 0.1});
  Eigen::Vector3f target_center_prior = target_center_camera;

  // handle y conditions
  std::function<bool(float)> y_condition;
  if (fabs(_req.until_y) > 999) // impossible goal, reject
  {
    y_condition = [=](float ref){ return false; };
  }
  else
  {
    if (_req.until_y >= 0)
      if (diff_to_object[1] > _req.until_y)
	y_condition = [=](float ref)
	{
	  if (ref > _req.until_y) // continue condition
	    return true;
	  return false; // escapes at false
	};
      else if (diff_to_object[1] < 0)
	y_condition = [=](float ref)
	{
	  if (ref < 0.0) // continue condition
	    return true;
	  return false; // escapes at false
	};
      else
	y_condition = [=](float ref){ return false; };
    else
      if (diff_to_object[1] < _req.until_y)
	y_condition = [=](float ref)
	{
	  if (ref < _req.until_y) // continue condition
	    return true;
	  return false; // escapes at false
	};
      else if (diff_to_object[1] > 0)
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

  while ((diff_to_object[0] > _req.until_x) ||
	 y_condition(diff_to_object[1]))
  {
    ROS_WARN("center: %f, %f, %f", sensor_->GetCenter().x(), sensor_->GetCenter().y(), sensor_->GetCenter().z());
    object_->ExtractObjectFeatures(
	sensor_->GetCenter(), sensor_->GetVertices(), sensor_->GetCloud());
    status = object_->GetStatus();

    if (status <= aero::status::aborted) // failed object detection
    {
      ac_->cancelGoal();
      _res.status = aero::status::fatal;
      return true;
    }

    base_to_eye = object_->GetBaseToEye();
    target_center_camera = object_->GetTargetCenterCamera();

    diff_to_object =
      Eigen::Vector3f(base_to_eye.getOrigin().x(),
                      base_to_eye.getOrigin().y(),
                      base_to_eye.getOrigin().z()) +
      base_to_eye_q * target_center_camera;

    sensor_->SetSpaceMax({target_center_camera[0], target_center_camera[1],
	  target_center_camera[2] - (target_center_prior[2] - target_center_camera[2]) + 0.1});

    target_center_prior = target_center_camera;

    auto time_now = aero::time::now();
    if (aero::time::ms(time_now - start) > 1000) // time_out
    {
      ac_->cancelGoal();
      _res.status = aero::status::fatal;
      return true;
    }
  }

  ac_->cancelGoal();
  auto end = aero::time::now();
  _res.time = aero::time::ms(end - start) / 1000.0;

  _res.status = aero::status::success;
  return true;
}
