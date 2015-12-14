#ifndef _AERO_PERCEPTION_POINT_CLOUD_SENSOR_H_
#define _AERO_PERCEPTION_POINT_CLOUD_SENSOR_H_

#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <aero_startup/PointXYZHSI.h>
#include "aero_common/types.h"

namespace aero
{
  namespace perception
  {

    class PointCloudSensor
    {
    public: explicit PointCloudSensor(ros::NodeHandle _nh);

    public: ~PointCloudSensor();

    protected: virtual void SubscribePoints(
        const sensor_msgs::PointCloud2::ConstPtr& _msg) = 0;

    protected: bool Reconfigure(aero_startup::PointXYZHSI::Request &_req,
				aero_startup::PointXYZHSI::Response &_res);

    public: bool GetTiming();

    public: Eigen::Vector3f GetCenter();

    public: std::vector<Eigen::Vector3f> GetVertices();

    public: pcl::PointCloud<pcl::PointXYZ>::Ptr GetCloud();

    public: std::vector<aero::rgb> GetRGB();

    public: void SetSpaceMax(aero::xyz _value);

    public: void SetSpaceMin(aero::xyz _value);

    public: void SetHSIMax(aero::hsi _value);

    public: void SetHSIMin(aero::hsi _value);

    protected: bool timing_;

    protected: Eigen::Vector3f center_;

    protected: std::vector<Eigen::Vector3f> vertices_;

    protected: pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    protected: std::vector<aero::rgb> rgb_;

    protected: aero::xyz space_min_;

    protected: aero::xyz space_max_;

    protected: aero::hsi target_hsi_max_;

    protected: aero::hsi target_hsi_min_;

    protected: ros::NodeHandle nh_;

    protected: ros::ServiceServer filter_service_;

    protected: ros::Publisher points_publisher_;
    };

    typedef std::shared_ptr<PointCloudSensor> PointCloudSensorPtr;

  }
}

#endif

/*
  @define srv
  float32 x_cap
  float32 y_cap
  float32 z_cap
  float32 x
  float32 y
  float32 z
  int8 h_cap
  uint8 s_cap
  uint8 i_cap
  int8 h
  uint8 s
  uint8 i
  bool precise
  ---
  int8 status
  bool prior_setting
*/
