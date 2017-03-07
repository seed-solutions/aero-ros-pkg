#ifndef _AERO_STD_TIME_
#define _AERO_STD_TIME_

#include <chrono>

namespace aero
{
  namespace time
  {
    inline std::chrono::high_resolution_clock::time_point now()
    { return std::chrono::high_resolution_clock::now(); };

    inline float ms(std::chrono::duration<double> _p)
    { return std::chrono::duration_cast<std::chrono::milliseconds>(_p).count(); };
  }
}

#endif
