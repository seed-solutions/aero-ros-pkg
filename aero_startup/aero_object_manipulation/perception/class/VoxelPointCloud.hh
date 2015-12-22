#ifndef _AERO_PERCEPTION_VOXEL_POINT_CLOUD_H_
#define _AERO_PERCEPTION_VOXEL_POINT_CLOUD_H_

#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include <pcl/filters/voxel_grid.h>

namespace aero
{
  namespace perception
  {

    class VoxelPointCloud : public PointCloudSensor
    {
    public: explicit VoxelPointCloud(ros::NodeHandle _nh);

    public: ~VoxelPointCloud();

    protected: void SubscribePoints(
        const sensor_msgs::PointCloud2::ConstPtr& _msg);

    protected: ros::Subscriber point_cloud_listener_;

    protected: ros::Publisher pcl_pub_;
    };

    typedef boost::shared_ptr<VoxelPointCloud> VoxelPointCloudPtr;

  }
}

#endif
