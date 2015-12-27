#ifndef _AERO_COMMON_AERO_
#define _AERO_COMMON_AERO_

namespace aero
{
  struct rgb
  {
    int r;
    int g;
    int b;
  };

  struct hsi
  {
    int h;
    int s;
    int i;
  };

  struct lab
  {
    float l;
    float a;
    float b;
  };

  struct xyz
  {
    float x;
    float y;
    float z;
  };

  struct box
  {
    xyz center;
    xyz max_bound;
    xyz min_bound;
    float points;
  };
}

#endif
