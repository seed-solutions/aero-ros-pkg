#ifndef _AERO_VISION_OBJECT_FEATURES_H_
#define _AERO_VISION_OBJECT_FEATURES_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#ifdef _CFG_DEPENDS_PCL_
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#endif

#ifdef _CFG_DEPENDS_OPENCV_
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif

#include <tf/transform_broadcaster.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <thread>
#include <mutex>

namespace aero
{
  namespace vision
  {

    class ObjectFeatures
    {
    public: explicit ObjectFeatures(ros::NodeHandle _nh);

    public: explicit ObjectFeatures(ros::NodeHandle _nh, std::string _frame);

    public: ~ObjectFeatures();

      // Convert vectors in camera coords to world coords.
    public: Eigen::Vector3f ConvertWorld(Eigen::Vector3f _pos_camera);

    //   // Convert point clouds in camera coords to world coords.
    // public: void ConvertWorld(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _in,
    //                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr _out);

    public: void Convert_mm(geometry_msgs::Point& _pos);

    public: inline geometry_msgs::Pose GetBaseToEye() { return base_to_eye_; };

    // public: void BroadcastTf(Eigen::Vector3f _position, std::string _name, std::string _frame);

    // public: void BroadcastTf(Eigen::Vector3f _position,
    //                          Eigen::Quaternionf _orientation,
    //                          std::string _name,
    //                          std::string _frame);

    public: void BroadcastTf(Eigen::Vector3f _position, std::string _name);

    public: void BroadcastTf(Eigen::Vector3f _position,
                             Eigen::Quaternionf _orientation,
                             std::string _name);

    // public: void BroadcastTf(Eigen::Vector3f _position, std::string _name, std::string _frame);

    // public: void BroadcastTf(Eigen::Vector3f _position,
    //                          Eigen::Quaternionf _orientation,
    //                          std::string _name,
    //                          std::string _frame);

    public: void StopBroadcastTfAll();

#ifdef _CFG_DEPENDS_PCL_
    public: void BroadcastPoints
    (pcl::PointCloud<pcl::PointXYZRGB>::Ptr _points, std::string _frame);

#endif

#ifdef _CFG_DEPENDS_OPENCV_
    public: void BroadcastImage(cv::Mat _img, std::string _frame);

    public: cv::Mat ResizeImage(cv::Mat _img, int _width, int _height);
#endif

    private: void SubscribeCameraPseudoTf
    (const geometry_msgs::Pose::ConstPtr& _pose);

    private: ros::NodeHandle nh_;

    private: ros::Subscriber camera_pseudo_tf_subscriber_;

    private: ros::CallbackQueue pseudo_tf_queue_;

    private: ros::AsyncSpinner pseudo_tf_spinner_;

    private: geometry_msgs::Pose base_to_eye_;

    private: std::mutex tf_mutex_;

    private: bool broadcast_tf_;

    private: ros::Publisher pcl_pub_;

    private: ros::Publisher img_pub_;
    };

    typedef std::shared_ptr<ObjectFeatures> ObjectFeaturesPtr;

  }
}

#endif
