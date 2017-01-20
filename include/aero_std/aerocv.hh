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
#include <pcl/segmentation/extract_clusters.h>

#include "aero_std/colors.h"
#include "aero_std/proc2d.h"

namespace aero
{
  namespace aerocv
  {

    /// @brief Identified information of object.
    struct objectproperties {
      std::vector<std::string> name;
      /// @brief List of named colors with percentage information.
      std::vector<std::pair<std::string, float> > colors;
    };

    /// @brief Information of detected object.
    struct objectarea {
      std::vector<int> indices3d; // compressed indices
      cv::Rect bounds2d;
      bool visible3d; // center3d, normal3d is null for such objects
      Eigen::Vector3f center3d; // used in post-process
      Eigen::Vector3f normal3d; // pre-calculated for post-process
      objectproperties properties; // only colors property is filled w/ detection
    };

    /// @brief Information of object in relation to environment.
    struct graspconfig {
      int sparse; // -1: detected facet, 0: detected nearby obstacle, 1: clear
      int contact; // -1: cannot grasp, 0: top grasp-able, 1: contact grasp
      std::vector<int> facets; // id of detected facets (for futher analysis)
    };

    /// @brief Detect area where objects likely exist with 3d region growing.
    ///   First, region growing segmentation is conducted.
    ///   Second, 2d clustering for non-detected area is conducted.
    ///   Third, 2d compression segmentation is conducted for large non-detected area.
    /// @param[in] _cloud Point cloud from sensor (low resolution for performance).
    /// @param[in] _img Image from sensor (for resolution ratio and drawing).
    /// @param[in] _env_color Color of environment to remove from object list.
    /// @param[in] _color_thre Color distance threshold to label as background color.
    /// @param[in] _debug_folder Folder to save process images. No save if empty.
    /// @return {List of detected objects, List of detected environment}
    std::pair<std::vector<objectarea>, std::vector<objectarea> > DetectObjectnessArea
    (pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,
     cv::Mat &_img, cv::Vec3b _env_color, float _color_thre, std::string _debug_folder="");

    /// @brief Sorts by color percentage and distance (best match comes first).
    ///   e.g. 100% blue is better match than 50% blue
    ///   e.g. 100% blue nearby is better match than 100% blue far away
    ///   e.g. 100% blue far away is better match than 50% blue nearby
    /// @param[in] _target_color Named color to find. cf. aerocv::colorMap9
    /// @param[in] _scene Lists of objects to find from.
    /// @return Indices (in vector _scene) of matched objects sorted by likeliness.
    std::vector<int> FindTarget
    (std::string _target_color, std::vector<objectarea> &_scene);

    /// @brief Filters list of objects with given indices.
    /// @param[in] _objects List of objects to filter.
    /// @param[in] _indices The indices of objects to retrieve.
    /// @return List of objects from applied indices.
    std::vector<objectarea> filter
    (std::vector<objectarea> _objects, std::vector<int> _indices);

    /// @brief Detect object information in relation to environment for grasping.
    /// @param[in] _target Id of target in vector _scene.
    /// @param[in] _scene List of objects to consider when analyzing.
    /// @param[in] _cloud Point cloud from sensor (used in DetectObjectnessArea).
    /// @param[in] _img Image from sensor (used in DetecObjectnessArea)
    /// @param[in] _debug_folder Folder to save process images. No save if empty.
    /// @return Likely required grasp strategy and detected object relations.
    graspconfig ConfigurationFromLocal1DState
    (int _target, std::vector<objectarea> &_scene,
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &_img,
     std::string _debug_folder="");

  }
}

#endif
