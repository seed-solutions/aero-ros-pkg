#ifndef _AERO_STD_INTERPOLATION_TYPE_
#define _AERO_STD_INTERPOLATION_TYPE_

#include "aero_startup/AeroInterpolation.h"

namespace aero
{
  namespace interpolation
  {
    static const int i_constant = 0;

    static const int i_linear = 1;

    static const int i_bezier = 2;

    static const int i_slowin = 3;

    static const int i_slowout = 4;

    static const int i_sigmoid = 5;

    static const int i_cbezier = 6;

    typedef aero_startup::AeroInterpolation::Request settings;
  }
}

#endif
