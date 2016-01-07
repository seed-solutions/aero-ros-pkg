#ifndef _AERO_COMMON_PERCEPTION_BASE_H_
#define _AERO_COMMON_PERCEPTION_BASE_H_

#include <ros/ros.h>
#include "geometry_msgs/Pose.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace aero
{
  namespace common
  {

    class Base
    {
    public: explicit Base(ros::NodeHandle _nh);

    public: ~Base();

    protected: void SubscribeCameraPseudoTf(
        const geometry_msgs::Pose::ConstPtr& _pose);

    protected: ros::NodeHandle nh_;

    protected: ros::Subscriber camera_pseudo_tf_subscriber_;

    protected: geometry_msgs::Pose base_to_eye_;
    };

  }
}

#endif
