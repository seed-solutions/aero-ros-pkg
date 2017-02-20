#ifndef _AERO_VISION_OBJECT_FEATURES_H_
#define _AERO_VISION_OBJECT_FEATURES_H_

#include <ros/ros.h>
#include <aero_std/AeroMoveitInterface.hh>

namespace aero
{
  namespace vision
  {

    class ObjectFeatures
    {
    public: explicit ObjectFeatures(ros::NodeHandle _nh);

    public: ~ObjectFeatures();

      // Convert vectors in camera coords to world coords.
    public: Eigen::Vector3d ConvertWorld(Eigen::Vector3d _pos_camera);

    public: void BroadcastTf(Eigen::Vector3f _position, std::string _name);

    public: void BroadcastTf(Eigen::Vector3f _position,
                             Eigen::Quaternionf _orientation,
                             std::string _name);

    public: void StopBroadcastTfAll();


    private: ros::NodeHandle nh_;

    private: aero::interface::AeroMoveitInterfacePtr interface_;
    };

    typedef std::shared_ptr<ObjectFeatures> ObjectFeaturesPtr;

  }
}

#endif
