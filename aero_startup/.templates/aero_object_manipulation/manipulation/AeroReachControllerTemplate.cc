/*
 * ThirdParty : libsvm/svm.h from https://github.com/cjlin1/libsvm
 */

#include "libsvm/svm.h"
#include <ros/ros.h>
#include <aero_startup/AeroReachControllerService.h>

/* implement here svm_model */

bool PredictReachPoint(aero_startup::AeroReachControllerService::Request &req,
		       aero_startup::AeroReachControllerService::Response &res)
{
  points data;

  /* implement here normalization */

  /* implement here to_input_data */

  /* implement here set_res */

  return true;
}

int main(int argc, char **argv)
{
  /* implement here svm_load */

  ros::init(argc, argv, "aero_reach_controller");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("aero_reach_controller", PredictReachPoint);
  ros::spin();

  return 0;
}
