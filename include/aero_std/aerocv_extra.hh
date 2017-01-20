#ifndef _AERO_STD_AEROCV_EXTRA_
#define _AERO_STD_AEROCV_EXTRA_

#include "aero_std/aerocv.hh"
#include "linux_kinect/WindowsInterface.hh"

namespace aero
{
  namespace aerocv
  {

    static const std::vector<std::vector<int> > laying_shear_x =
      {{15, 10, 0, -10, -15},
       {20, 10, 0, -10, -20},
       {25, 15, 0, -15, -25},
       {30, 20, 0, -20, -30},
       {40, 25, 0, -25, -40}};

    static const std::vector<std::vector<int> > standing_shear_x =
      {{-15, -10, 0, 10, 15},
       {-15, -10, 0, 10, 15},
       {-20, -15, 0, 15, 20},
       {-25, -20, 0, 20, 25},
       {-30, -20, 0, 20, 30}};

    static const std::vector<std::vector<int> > laying_transform_y =
      {{0, 0, 0, 0, 0},
       {0, 0, 0, 0, 0},
       {0, 0, 0, 0, 0},
       {0, 0, 0, 0, 0},
       {0, 0, 0, 0, 0}};

    static const std::vector<std::vector<int> > standing_transform_y =
      {{0, 0, 0, 0, 0},
       {0, 0, 0, 0, 0},
       {0, 0, 0, 0, 0},
       {0, 0, 0, 0, 0},
       {28, 28, 28, 28, 28}};

    std::vector<int> FindTargetWithOcr
    (std::vector<std::string> _target_name, std::vector<objectarea> &_scene, cv::Mat &_img,
     windows::interface::WindowsInterfacePtr _windows, std::string _debug_folder="");

  }
}

#endif
