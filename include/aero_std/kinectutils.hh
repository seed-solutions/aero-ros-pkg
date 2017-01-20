#ifndef _AERO_STD_KINECT_UTILS_
#define _AERO_STD_KINECT_UTILS_

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "aero_std/aerocv.hh"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

namespace kinect
{
  namespace interface
  {

    //////////////////////////////////////////////////
    void GetCloud(sensor_msgs::PointCloud2 &points,
                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
      // ros msg -> pcl PointCloud
      pcl::PCLPointCloud2 pcl;
      pcl_conversions::toPCL(points, pcl);
      pcl::fromPCLPointCloud2(pcl, *cloud);

      // flip x, flip y (= xtion coordinate), indices must also be flipped
      int row = 0;
      for (auto p = cloud->points.begin(), q = p + cloud->width - 1; ; ) {
        p->x = -p->x; p->y = -p->y;
        q->x = -q->x; q->y = -q->y;
        auto tmp = *q;
        *q = *p;
        *p = tmp;
        if (q == p + 1) {
          p = cloud->points.begin() + cloud->width * ++row;
          if (p == cloud->points.end()) break;
          q = p + cloud->width - 1;
        } else {
          ++p;
          --q;
        }
      }
    };

    //////////////////////////////////////////////////
    void GetImage(sensor_msgs::Image &image, cv::Mat &img)
    {
      // get image as cv::Mat
      int k = 3;
      for (unsigned int i = 0; i < img.rows; ++i)
        for (int j = img.cols - 1; j >= 0; --j) { // flip x
          img.at<cv::Vec3b>(i, j) =
            cv::Vec3b(image.data[--k], image.data[--k], image.data[--k]);
          k += 6;
        }
    };

    //////////////////////////////////////////////////
    void Convert(aero::aerocv::objectarea &obj)
    {
      // indices will not be flipped
      // 2d data will also not be flipped

      // flip x, flip y for 3d data
      obj.center3d =
        Eigen::Vector3f(-obj.center3d.x(), -obj.center3d.y(), obj.center3d.z());
      obj.normal3d =
        Eigen::Vector3f(-obj.normal3d.x(), -obj.normal3d.y(), obj.normal3d.z());
    };

  }
}

#endif
