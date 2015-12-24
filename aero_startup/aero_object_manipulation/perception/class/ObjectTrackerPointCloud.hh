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

namespace aero
{
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

    protected: ros::Subscriber point_cloud_listener_;

    protected: aero::box object_;

    protected: ros::ServiceClient get_object_;
    };

    typedef std::shared_ptr<ObjectTrackerPointCloud> ObjectTrackerPointCloudPtr;

  }
}

#endif
