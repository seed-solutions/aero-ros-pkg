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
#include <aero_startup/ReturnRGB.h>
#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_common/types.h"
#include "aero_common/colors.h"
#include "aero_common/time.h"
#include <pcl/filters/statistical_outlier_removal.h>

#ifdef CXX11_SUPPORTED
#include <random>
#else
#include <boost/random.hpp>
#include <boost/random/random_device.hpp>
#endif

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

    protected: bool ReturnPlaneRGB(aero_startup::ReturnRGB::Request  &_req,
				   aero_startup::ReturnRGB::Response &_res);

    protected: ros::Subscriber point_cloud_listener_;

    protected: Eigen::Vector3f desk_plane_norm_;

    protected: float height_range_per_region_;

    protected: ros::Publisher pcl_pub_;

    protected: ros::ServiceServer return_plane_color_service_;

    protected: aero::rgb plane_color_;
    };

#ifdef CXX11_SUPPORTED
    typedef std::shared_ptr<PlaneDetectedPointCloud> PlaneDetectedPointCloudPtr;
#else
    typedef boost::shared_ptr<PlaneDetectedPointCloud> PlaneDetectedPointCloudPtr;
#endif

  }
}

#endif

/*
  @define srv
  ---
  int32 r
  int32 g
  int32 b
*/
