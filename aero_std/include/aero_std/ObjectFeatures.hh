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
    public: explicit ObjectFeatures(ros::NodeHandle _nh);

    public: explicit ObjectFeatures(ros::NodeHandle _nh, aero::interface::AeroMoveitInterface::Ptr _interface);

    public: ~ObjectFeatures();

      // Convert vectors in camera coords to world coords.
    public: Eigen::Vector3d convertWorld(Eigen::Vector3d _pos, bool _update_model=true);
    public: Eigen::Vector3d convertWorld(Eigen::Vector3f _pos, bool _update_model=true);

    public: void setCameraTransform(std::string _camera_parent_link, Eigen::Vector3d _position, Eigen::Quaterniond _orientation);

    public: int setMarker(Eigen::Vector3d _position, Eigen::Quaterniond _orientation, int _id=1);
    public: int setMarker(Eigen::Vector3d _position, int _id=1);
    public: int setMarker(Eigen::Vector3f _position, int _id=1);
    public: int setMarker(geometry_msgs::Pose _pose, int _id=1);
    public: int setMarker(Eigen::Vector3d _pos1, Eigen::Vector3d _pos2, int _id=1);
    public: int setMarker(Eigen::Vector3f _pos1, Eigen::Vector3f _pos2, int _id=1);

    public: int setMesh(geometry_msgs::Pose _pose, std::string _mesh_path, int _id=1, std_msgs::ColorRGBA _color=std_msgs::ColorRGBA());

    private: ros::NodeHandle nh_;

    private: aero::interface::AeroMoveitInterface::Ptr interface_;

    private: ros::Publisher marker_publisher_;

    private: std::string camera_parent_link_;
    private: Eigen::Vector3d camera_relative_position_;
    private: Eigen::Quaterniond camera_relative_orientation_;
    };

    typedef std::shared_ptr<ObjectFeatures> ObjectFeaturesPtr;

  }
}

#endif
