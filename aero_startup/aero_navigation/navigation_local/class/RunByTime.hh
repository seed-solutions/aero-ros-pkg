#ifndef _AERO_NAVIGATION_RUN_BY_TIME_H_
#define _AERO_NAVIGATION_RUN_BY_TIME_H_

#include "aero_navigation/navigation_local/class/Runner.hh"
#include "aero_common/status.h"
#include "aero_common/time.h"
#include <aero_startup/PointGoTime.h>

namespace aero
{
  namespace navigation
  {

    class RunByTime : Runner
    {
    public: explicit RunByTime(ros::NodeHandle _nh);

    public: ~RunByTime();

    private: bool GoForTime(aero_startup::PointGoTime::Request  &_req,
			    aero_startup::PointGoTime::Response &_res);

    private: ros::ServiceServer goal_service_;
    };

    typedef std::shared_ptr<RunByTime> RunByTimePtr;

  }
}

#endif

/*
  @define srv
  float32 go_x
  float32 go_y
  float32 go_theta
  float32 time
  ---
  int8 status
*/
