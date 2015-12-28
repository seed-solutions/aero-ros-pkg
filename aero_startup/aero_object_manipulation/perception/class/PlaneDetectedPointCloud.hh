#ifndef _AERO_PERCEPTION_PLANE_DETECTION_H_
#define _AERO_PERCEPTION_PLANE_DETECTION_H_

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_common/types.h"
#include "aero_common/colors.h"

#include <random>

namespace aero
{
  namespace perception
  {

    class PlaneDetectedPointCloud : public PointCloudSensor
    {
    public: explicit PlaneDetectedPointCloud(ros::NodeHandle _nh);

    public: ~PlaneDetectedPointCloud();

    protected: void SubscribePoints(
	const sensor_msgs::PointCloud2::ConstPtr& _msg);

    protected: ros::Subscriber point_cloud_listener_;

    protected: Eigen::Vector3f desk_plane_norm_;

    protected: float height_range_per_region_;

    protected: ros::Publisher pcl_pub_;
    };

    typedef std::shared_ptr<PlaneDetectedPointCloud> PlaneDetectedPointCloudPtr;

  }
}

#endif
