#ifndef _AERO_PERCEPTION_XYZHSI_POINT_CLOUD_H_
#define _AERO_PERCEPTION_XYZHSI_POINT_CLOUD_H_

#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"

namespace aero
{
  namespace perception
  {

    class XYZHSIPointCloud : public PointCloudSensor
    {
    public: explicit XYZHSIPointCloud(ros::NodeHandle _nh);

    public: ~XYZHSIPointCloud();

    protected: void SubscribePoints(
	const sensor_msgs::PointCloud2::ConstPtr& _msg);

    protected: bool ValidHSI(aero::rgb _color);

    protected: ros::Subscriber point_cloud_listener_;

    // protected: ros::Publisher pcl_pub_;
    };

    typedef std::shared_ptr<XYZHSIPointCloud> XYZHSIPointCloudPtr;

  }
}

#endif
