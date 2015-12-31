#ifndef _AERO_NAVIGATION_RUN_BY_VISION_H_ 
#define _AERO_NAVIGATION_RUN_BY_VISION_H_

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include "aero_navigation/navigation_local/class/Runner.hh"
#include "aero_common/types.h"
#include "aero_common/status.h"
#include "aero_common/time.h"
#include <aero_startup/TimeFromGoXY.h>
#include <aero_startup/ProcessSleep.h>

namespace aero
{
  namespace navigation
  {

    class RunByVision : Runner
    {
    public: explicit RunByVision(ros::NodeHandle _nh);

    public: ~RunByVision();

    private: void Subscribe(const geometry_msgs::Pose::ConstPtr& _object);

    private: bool GoForTarget(aero_startup::TimeFromGoXY::Request &_req,
			      aero_startup::TimeFromGoXY::Response &_res);

    private: ros::ServiceServer goal_service_;

    private: ros::Subscriber object_subscriber_;

    private: ros::ServiceClient recognition_mode_;

    private: ros::ServiceClient tracking_mode_;

    private: aero::xyz object_;
    };

    typedef std::shared_ptr<RunByVision> RunByVisionPtr;

  }
}

#endif

/*
  @define srv
  string object
  float32 go_x
  float32 go_y
  float32 until_x
  float32 until_y
  float32 time_out_ms
  ---
  int8 status
  float32 time
*/
