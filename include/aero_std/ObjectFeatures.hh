#ifndef _AERO_VISION_OBJECT_FEATURES_H_
#define _AERO_VISION_OBJECT_FEATURES_H_

#include <ros/ros.h>
#include <aero_std/AeroMoveitInterface.hh>
#include <visualization_msgs/Marker.h>

namespace aero
{
  namespace vision
  {

    class ObjectFeatures
    {
    public: explicit ObjectFeatures(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _interface);

    public: ~ObjectFeatures();

      // Convert vectors in camera coords to world coords.
    public: Eigen::Vector3d ConvertWorld(Eigen::Vector3d _pos);

    public: void setCameraTransform(std::string _camera_parent_link, Eigen::Vector3d _position, Eigen::Quaterniond _orientation);

    public: int setMarker(geometry_msgs::Point _position, int _id=1);

    private: ros::NodeHandle nh_;

    private: aero::interface::AeroMoveitInterfacePtr interface_;

    private: ros::Publisher marker_publisher_;

    private: std::string camera_parent_link_;
    private: Eigen::Vector3d camera_relative_position_;
    private: Eigen::Quaterniond camera_relative_orientation_;
    };

    typedef std::shared_ptr<ObjectFeatures> ObjectFeaturesPtr;

  }
}

#endif
