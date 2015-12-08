#ifndef _AERO_NAVIGATION_RUN_BY_VISION_H_ 
#define _AERO_NAVIGATION_RUN_BY_VISION_H_

#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_object_manipulation/perception/class/ObjectFeatures.hh"
#include "aero_navigation/navigation_local/class/Runner.hh"
#include "aero_common/status.h"
#include "aero_common/time.h"
#include <aero_startup/ObjectGoXYZHSI.h>

namespace aero
{
  namespace navigation
  {

    class RunByVision : Runner
    {
    public: explicit RunByVision(ros::NodeHandle _nh,
				 perception::PointCloudSensorPtr _sensor,
				 perception::ObjectFeaturesPtr _object);

    public: ~RunByVision();

    private: bool GoForTarget(aero_startup::ObjectGoXYZHSI::Request &_req,
			      aero_startup::ObjectGoXYZHSI::Response &_res);

    private: perception::PointCloudSensorPtr sensor_;

    private: perception::ObjectFeaturesPtr object_;

    private: ros::ServiceServer goal_service_;
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
  ---
  int8 status
  float32 time
*/
