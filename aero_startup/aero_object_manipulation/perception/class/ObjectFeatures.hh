#ifndef _AERO_PERCEPTION_OBJECT_FEATURES_H_
#define _AERO_PERCEPTION_OBJECT_FEATURES_H_

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "aero_object_manipulation/perception/class/Base.hh"
#include <Eigen/SVD>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <tf/transform_broadcaster.h>
#include "aero_common/types.h"
#include "aero_common/status.h"
#include "aero_common/time.h"

namespace aero
{
  namespace perception
  {

    class ObjectFeatures : public aero::common::Base
    {
    public: explicit ObjectFeatures(ros::NodeHandle _nh);

    public: ~ObjectFeatures();

    public: void ExtractObjectFeatures(
	std::vector<Eigen::Vector3f> _vertices);

    protected: void Subscribe(
        const std_msgs::Float32MultiArray::ConstPtr& _points);

    protected: void BroadcastTf();

    protected: void BroadcastPose();

    public: int GetStatus();

    public: geometry_msgs::Pose GetBaseToEye();

    public: Eigen::Vector3f GetTargetCenterCamera();

    protected: int status_;

    protected: Eigen::Vector3f target_center_camera_;

    protected: Eigen::Quaternionf target_pose_camera_;

    protected: Eigen::Vector3f target_center_world_;

    protected: Eigen::Quaternionf target_pose_world_left_;

    protected: Eigen::Quaternionf target_pose_world_right_;

    protected: int lost_count_;

    protected: static const int lost_threshold_ = 30;

    protected: ros::Publisher pose_publisher_left_;

    protected: ros::Publisher pose_publisher_right_;

    protected: ros::Subscriber subscriber_;

    protected: int target_;
    };

    typedef std::shared_ptr<ObjectFeatures> ObjectFeaturesPtr;

  }
}

#endif
