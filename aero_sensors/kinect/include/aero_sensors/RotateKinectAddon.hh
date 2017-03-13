#ifndef _AERO_SENSORS_ROTATE_KINECT_ADDON_
#define _AERO_SENSORS_ROTATE_KINECT_ADDON_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <mutex>

#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aero
{
  namespace addon
  {

    class RotateKinectAddon
    {
    public: explicit RotateKinectAddon(ros::NodeHandle _nh);

    public: ~RotateKinectAddon();

    public: void rotateKinectTo(int _angle);

    public: std::pair<Eigen::Vector3f, Eigen::Quaternionf>
    getBaseToEye();

    private: void SubscribeCameraPseudoTf
    (const geometry_msgs::Pose::ConstPtr& _pose);

    private: ros::NodeHandle nh_;

    private: ros::Publisher kinect_control_publisher_;

    private: ros::Subscriber camera_pseudo_tf_subscriber_;

    private: ros::CallbackQueue pseudo_tf_queue_;

    private: ros::AsyncSpinner pseudo_tf_spinner_;

    private: geometry_msgs::Pose base_to_eye_;

    private: std::mutex tf_mutex_;
    };

    typedef std::shared_ptr<RotateKinectAddon> RotateKinectAddonPtr;

  }
}

#endif
