#ifndef _AERO_PERCEPTION_OBJECT_FEATURES_H_
#define _AERO_PERCEPTION_OBJECT_FEATURES_H_

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Core>
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

    class ObjectFeatures
    {
    public: ObjectFeatures();

    public: ~ObjectFeatures();

    public: void ExtractObjectFeatures(
	Eigen::Vector3f _center, std::vector<Eigen::Vector3f> _vertices,
	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud);

    private: void BroadcastTf();

    public: int GetStatus();

    public: tf::StampedTransform GetBaseToEye();

    public: Eigen::Vector3f GetTargetCenterCamera();

    protected: int status_;

    protected: Eigen::Vector3f target_center_camera_;

    protected: Eigen::Quaternionf target_pose_camera_;

    protected: Eigen::Vector3f target_center_world_;

    protected: Eigen::Quaternionf target_pose_world_left_;

    protected: Eigen::Quaternionf target_pose_world_right_;

    protected: tf::StampedTransform base_to_eye_;

    protected: int lost_count_;

    protected: static const int lost_threshold_ = 30;
    };

    typedef std::shared_ptr<ObjectFeatures> ObjectFeaturesPtr;

  }
}

#endif
