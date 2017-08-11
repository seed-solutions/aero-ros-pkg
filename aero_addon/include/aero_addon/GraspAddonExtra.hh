#ifndef _AERO_ADDON_GRASP_ADDON_EXTRA_
#define _AERO_ADDON_GRASP_ADDON_EXTRA_

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "aero_addon/GraspAddon.hh"
#include "aero_std/ObjectFeatures.hh"
#include "aero_sensors/RotateKinectAddon.hh"
#include "linux_kinect/KinectInterface.hh"

namespace aero
{
  namespace addon
  {

    struct tumbleparameters {
      tumbleparameters() {
        arm = aero::arm::larm;
        depth_thre = 0.1f;
        r = 0.025f;
        level_margin = 0.01f;
        d = 0.02f;
        omega = -0.52359877f;
        shortcut = 0;
      };

      aero::arm arm;
      float depth_thre;
      float r;
      float level_margin;
      float d;
      float omega;
      int shortcut;
    };

    class GraspAddonExtra : public GraspAddon
    {
    public: explicit GraspAddonExtra
    (ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _robot,
     aero::vision::ObjectFeaturesPtr _features,
     aero::addon::RotateKinectAddonPtr _kcon,
     kinect::interface::KinectInterfacePtr _kinect);

    public: ~GraspAddonExtra();

    // _target must be world coordinates
    public: bool sendBoxTumbleInitialLifter(Eigen::Vector3d _target);

    // _target must be world coordinates
    // _bounds2d must be in Kinect depth coordinates
    // _cloud must be in Kinect coordinates
    public: bool findBoxTumbleParameters
    (Eigen::Vector3d _target, float _width, float _height, cv::Rect _bounds2d,
     std::string _file,
     std::string _dbgfolder="", tumbleparameters _params=tumbleparameters());

    private: float tmbGetDepthOfTarget
    (float _seed_depth_index,
     float _roi_above, float _roi_below, float roi_left, float roi_right,
     float _initial_lifter_z, std::string _dbgfolder="", int _savenum=0);

    private: bool tmbFindABBoundIdx
    (int &_idx, float _ref, float _stride,
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

    private: bool tmbFindLRBoundIdx
    (int &_idx, int _row_start, int _row_end, float _ref,
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);

    private: aero::vision::ObjectFeaturesPtr features_;

    private: aero::addon::RotateKinectAddonPtr kcon_;

    private: kinect::interface::KinectInterfacePtr kinect_;

    private: float resize_x_;

    private: float resize_y_;

    private: float tmb_initial_lifter_z_;
    };

    typedef std::shared_ptr<GraspAddonExtra> GraspAddonExtraPtr;
  }
}

#endif
