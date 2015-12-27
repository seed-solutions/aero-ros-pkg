#ifndef _AERO_PERCEPTION_OBJECT_TRACKER_H_
#define _AERO_PERCEPTION_OBJECT_TRACKER_H_

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <aero_startup/BoxFromXYZ.h>
#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_common/types.h"
#include "aero_common/status.h"
#include "aero_common/time.h"
#include "aero_common/colors.h"
#include <tf/transform_broadcaster.h>
#include <random>

namespace aero
{
  struct rgb_dist
  {
    rgb color;
    float dist;

    bool operator < (const rgb_dist& _p) const
    {
      return (dist < _p.dist);
    }
  };

  namespace perception
  {

    class ObjectTrackerPointCloud : public PointCloudSensor
    {
    public: explicit ObjectTrackerPointCloud(ros::NodeHandle _nh);

    public: ~ObjectTrackerPointCloud();

    protected: void SubscribePoints(
        const sensor_msgs::PointCloud2::ConstPtr& _msg);

    protected: bool ProcessSleep(aero_startup::ProcessSleep::Request &_req,
				 aero_startup::ProcessSleep::Response &_res);

    protected: void InitializeTrackingObjectData(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _raw);

    protected: inline bool BoundingConditions(Eigen::Vector3f _p)
      {
	if ((_p.x() > object_.min_bound.x) &&
	    (_p.x() < object_.max_bound.x) &&
	    (_p.y() > object_.min_bound.y) &&
	    (_p.y() < object_.max_bound.y) &&
	    (_p.z() > object_.min_bound.z) &&
	    (_p.z() < object_.max_bound.z))
	  return true;
	return false;
      };

    protected: void BroadcastTf();

    protected: ros::Subscriber point_cloud_listener_;

    protected: aero::box object_;

    protected: ros::ServiceClient get_object_;

    protected: Eigen::Vector3f object_center_;

    protected: aero::hsi trim_color_min_;

    protected: aero::hsi trim_color_max_;
    };

    typedef std::shared_ptr<ObjectTrackerPointCloud> ObjectTrackerPointCloudPtr;

  }
}

#endif
