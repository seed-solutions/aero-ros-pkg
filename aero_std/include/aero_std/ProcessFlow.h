#ifndef _AERO_PROCESS_FLOW_H_
#define _AERO_PROCESS_FLOW_H_

#include "aero_std/GraspRequest.hh"

namespace aero
{
  template<typename T1, typename T2> T2 MyPerception(T1 _scene) {
    return T2();
  };

  template<typename T> GraspRequest Grasp(T _grasp) {
    return GraspRequest();
  };
}

#endif
