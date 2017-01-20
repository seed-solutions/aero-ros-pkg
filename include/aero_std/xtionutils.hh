#ifndef _AERO_STD_XTION_UTILS_
#define _AERO_STD_XTION_UTILS_

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

namespace xtion
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
    };

    //////////////////////////////////////////////////
    void GetImage(sensor_msgs::Image &image, cv::Mat &img)
    {
      // get image as cv::Mat
      int k = 0;
      for (unsigned int i = 0; i < img.rows; ++i)
        for (unsigned int j = 0; j < img.cols; ++j)
          img.at<cv::Vec3b>(i, j) =
            cv::Vec3b(image.data[k++], image.data[k++], image.data[k++]);
    };

    //////////////////////////////////////////////////
    void Convert(aero::aerocv::objectarea &obj)
    {
      printf("xtion does not require convert!");
    };

  }
}

#endif
