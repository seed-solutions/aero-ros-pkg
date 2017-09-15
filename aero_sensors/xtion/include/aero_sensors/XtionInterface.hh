#ifndef _AERO_SENSORS_XTION_INTERFACE_
#define _AERO_SENSORS_XTION_INTERFACE_

#include <ros/ros.h>

#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "geometry_msgs/Point.h"

#include <mutex>
#include <cstring>
#include <cstdint>

#include <limits>
#include <cmath>

namespace xtion
{
  namespace interface
  {

    class XtionInterface
    {
    public: explicit XtionInterface(ros::NodeHandle _nh);

    public: ~XtionInterface();

    public: sensor_msgs::PointCloud2 ReadPoints();

    public: sensor_msgs::PointCloud2 ReadPoints(float _scale_x, float _scale_y);

    public: sensor_msgs::Image ReadImage();

    public: sensor_msgs::PointCloud2 ReadPointsAfter(float _scale_x, float _scale_y);

    public: sensor_msgs::Image ReadImageAfter();

    public: std::vector<sensor_msgs::RegionOfInterest> ImageBounds
    (std::vector<std::array<int, 4> > _depth_indicies);

    public: std::vector<sensor_msgs::RegionOfInterest> ImageBounds
    (std::vector<std::array<int, 4> > _depth_indicies, float _w_scale, float _h_scale);

    public: std::vector<geometry_msgs::Point> ImageCenters
    (std::vector<sensor_msgs::RegionOfInterest> _image_bounds);

    public: void SetNow();

    private: void DepthCallback(const sensor_msgs::PointCloud2::ConstPtr& _msg);

    private: void ImageCallback(const sensor_msgs::Image::ConstPtr& _msg);

    private: sensor_msgs::PointCloud2 depth_;

    private: std::mutex depth_mutex_;

    private: sensor_msgs::Image image_;

    private: std::mutex image_mutex_;

    private: ros::NodeHandle nh_;

    private: ros::Subscriber depth_sub_;

    private: ros::SubscribeOptions depth_ops_;

    private: ros::CallbackQueue depth_queue_;

    private: ros::AsyncSpinner depth_spinner_;

    private: ros::Subscriber image_sub_;

    private: ros::SubscribeOptions image_ops_;

    private: ros::CallbackQueue image_queue_;

    private: ros::AsyncSpinner image_spinner_;

    private: int depth_width_;

    private: int depth_height_;

    private: ros::Time time_now_;
    };

    typedef std::shared_ptr<XtionInterface> XtionInterfacePtr;

  }
}

#endif
