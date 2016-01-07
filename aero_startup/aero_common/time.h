#ifndef _AERO_COMMON_TIME_
#define _AERO_COMMON_TIME_

#ifdef CXX11_SUPPORTED
#include <chrono>
#else
#include <boost/chrono.hpp>
#endif

namespace aero
{
  namespace time
  {
#ifdef CXX11_SUPPORTED
    inline std::chrono::high_resolution_clock::time_point now()
    { return std::chrono::high_resolution_clock::now(); };

    inline float ms(std::chrono::duration<double> _p)
    { return std::chrono::duration_cast<std::chrono::milliseconds>(_p).count(); };
#else
    inline boost::chrono::system_clock::time_point now()
    { return boost::chrono::system_clock::now(); };

    inline float ms(boost::chrono::duration<double> _p)
    { return boost::chrono::duration_cast<boost::chrono::milliseconds>(_p).count(); };
#endif
  }
}

#endif
