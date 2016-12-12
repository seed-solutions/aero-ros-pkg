#ifndef _AERO_PROCESS_FLOW_H_
#define _AERO_PROCESS_FLOW_H_

#include "aero_msgs/GraspIK.h"

namespace aero
{
  template<typename T1, typename T2> T2 MyPerception(T1 _scene) {
    return T2();
  };

  template<typename T> aero_msgs::GraspIK::Request MyGrasp(T _grasp) {
    return aero_msgs::GraspIK::Request();
  };
}

#endif
