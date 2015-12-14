#ifndef _AERO_COMMON_CLUSTERED_CLOUD_H_
#define _AERO_COMMON_CLUSTERED_CLOUD_H_

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Core>
#include "aero_common/types.h"

namespace aero
{
  namespace common
  {

    class ClusteredCloud
    {
    public: explicit ClusteredCloud(ros::NodeHandle _nh);

    public: ~ClusteredCloud();

    public: void Spin();

    protected: void Subscribe(
        const std_msgs::Float32MultiArray::ConstPtr& _points);

    public: void SetPublishStyle(std::function<void()> _func);

    public: void PublishAllClusters();

    public: void PublishLargestCluster();

    protected: std::vector<std::vector<Eigen::Vector3f> > clouds_;

    protected: ros::NodeHandle nh_;

    protected: ros::Publisher point_publisher_;

    protected: ros::Subscriber subscriber_;

    protected: std::function<void()> publish_func_;
    };

    typedef std::shared_ptr<ClusteredCloud> ClusteredCloudPtr;

  }
}

#endif
