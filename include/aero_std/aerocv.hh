#ifndef _AERO_STD_AEROCV_
#define _AERO_STD_AEROCV_

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/region_growing.h>

#include "aero_std/colors.h"
#include "aero_std/proc2d.h"

namespace aero
{
  namespace aerocv
  {

    struct objectproperties {
      std::string name;
      std::vector<std::pair<std::string, float> > colors;
    };

    struct objectarea {
      std::vector<int> indices3d; // compressed indices
      cv::Rect bounds2d;
      bool visible3d; // center3d, normal3d is null for such objects
      Eigen::Vector3f center3d; // used in post-process
      Eigen::Vector3f normal3d; // pre-calculated for post-process
      objectproperties properties; // only colors property is filled w/ detection

      // additional info for post-process
      // (calculated unless for speed optimization)
      std::array<cv::Point2f, 4> corners2d; // used for OCR, ...
      // corners2d coordinate is local to bounds
      float width3d; // used for OCR, grasp utils, ...
      float height3d; // used for OCR, grasp utils, ...
    };

    struct graspconfig {
      bool sparse;
      bool contact;
    };

    std::vector<objectarea> DetectObjectnessArea
    (pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,
     cv::Mat &_img, cv::Vec3b _env_color, bool _debug_view = true);

  }
}

#endif
