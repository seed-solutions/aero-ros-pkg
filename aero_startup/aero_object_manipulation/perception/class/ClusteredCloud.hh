#ifndef _AERO_COMMON_CLUSTERED_CLOUD_H_
#define _AERO_COMMON_CLUSTERED_CLOUD_H_

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Core>
#include <aero_startup/BoxFromXYZ.h>
#include "aero_common/types.h"
#include "aero_common/time.h"

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

    protected: void SubscribeClusters(
        const std_msgs::Float32MultiArray::ConstPtr& _clusters);

    protected: bool CallClusterFromCenter(
        aero_startup::BoxFromXYZ::Request &_req,
	aero_startup::BoxFromXYZ::Response &_res);

    public: void SetPublishStyle(std::function<void()> _func);

    public: void PublishStop();

    public: void PublishAllClusters();

    public: void PublishLargestCluster();

    protected: std::vector<std::vector<Eigen::Vector3f> > clouds_;

    protected: std::vector<aero::box> clusters_;

    protected: std::function<void()> publish_func_;

    // @brief : function to set when subscribe is reactivated
    protected: std::function<void()> default_func_;

    protected: bool process_sleep_;

    protected: std::chrono::high_resolution_clock::time_point sleep_start_;

    protected: ros::NodeHandle nh_;

    protected: ros::Publisher point_publisher_;

    protected: ros::Subscriber points_subscriber_;

    protected: ros::Subscriber cluster_subscriber_;

    protected: ros::ServiceServer return_cluster_from_center_;
    };

    typedef std::shared_ptr<ClusteredCloud> ClusteredCloudPtr;

  }
}

#endif

/*
  @define srv
  float32 x
  float32 y
  float32 z
  bool kill_spin
  ---
  float32 c_x
  float32 c_y
  float32 c_z
  float32 max_x
  float32 max_y
  float32 max_z
  float32 min_x
  float32 min_y
  float32 min_z
  int32 points
*/
