#ifndef _AERO_STD_PROC2D_
#define _AERO_STD_PROC2D_

#include <vector>
#include <climits>

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace aero
{
  namespace aerocv
  {

    //////////////////////////////////////////////////
    int cutComponentsWithStats
      (cv::Mat image, cv::Mat &labels, cv::Mat &stats, cv::Mat &centroids,
       bool debug_view = false)
    {
      // if image is too small to resize, return
      if (image.cols < 8 || image.rows < 8) return 1;

      // stretch image horizontally
      cv::Mat image_stretchedin(image.rows, image.cols * 4, CV_8U);
      cv::resize(image, image_stretchedin, image_stretchedin.size());
      cv::Mat image_stretched(image.rows, image.cols * 4, CV_8U);
      cv::threshold(image_stretchedin, image_stretched, 250, 255, 0);

      // compress image to sizexsize, this will divide thinly connected clusters
      cv::Mat image8x8in(8, 8, CV_8U);
      cv::resize(image_stretched, image8x8in, image8x8in.size());

      cv::Mat image8x8(8, 8, CV_8U);
      cv::threshold(image8x8in, image8x8, 150, 255, 0);
      cv::Mat labeled8x8;
      cv::Mat stats8x8;
      cv::Mat centroids8x8;
      int n8x8 =
        cv::connectedComponentsWithStats(
            image8x8, labeled8x8, stats8x8, centroids8x8);

      // if cluster is not further dividable
      if (n8x8 <= 2) return 1;

      if (debug_view) {
        std::string window_name =
          std::to_string(image.cols) + "x" + std::to_string(image.rows);

        cv::namedWindow(window_name, CV_WINDOW_NORMAL);
        cv::imshow(window_name, image);
        cv::waitKey(100);

        cv::namedWindow(window_name + "stretched", CV_WINDOW_NORMAL);
        cv::imshow(window_name + "stretched", image_stretchedin);
        cv::waitKey(100);

        cv::namedWindow(window_name + "thre", CV_WINDOW_NORMAL);
        cv::imshow(window_name + "thre", image8x8in);
        cv::waitKey(100);
      }

      // get centroids in original image size
      double r_w = image.cols / 8.0;
      double r_h = image.rows / 8.0;
      centroids = cv::Mat::zeros(n8x8, 2, CV_64F);
      for (int l = 1; l < n8x8; ++l) {
        centroids.at<double>(l, 0) =
          centroids8x8.at<double>(l, 0) * r_w;
        centroids.at<double>(l, 1) =
          centroids8x8.at<double>(l, 1) * r_h;
      }

      // get labels (search nearest center for each pixel)
      labels = cv::Mat::zeros(image.rows, image.cols, CV_32SC1);
      stats = cv::Mat::zeros(n8x8, 6, CV_32S);
      std::vector<std::array<int, 4> > bounds(n8x8, {image.cols, image.rows, 0, 0});
      for (unsigned int i = 0; i < image.rows; ++i)
        for (unsigned int j = 0; j < image.cols; ++j)
          if (image.at<uchar>(i, j) == 255) {
            double min_dist = std::numeric_limits<double>::max();
            int nearest_c = 0;
            for (int k = 1; k < centroids.rows; ++k) {
              double dist = std::pow(centroids.at<double>(k, 0) - j, 2)
                + std::pow(centroids.at<double>(k, 1) - i, 2);
              if (dist < min_dist) {
                min_dist = dist;
                nearest_c = k;
              }
            }
            labels.at<int>(i, j) = nearest_c;
            int *param = stats.ptr<int>(nearest_c);
            param[cv::ConnectedComponentsTypes::CC_STAT_AREA] += 1;
            auto b = bounds.begin() + nearest_c;
            if (j < b->at(0)) b->at(0) = j;
            else if (j > b->at(2)) b->at(2) = j;
            if (i < b->at(1)) b->at(1) = i;
            else if (i > b->at(3)) b->at(3) = i;
          }

      // write bounds to stats
      for (auto it = bounds.begin() + 1; it != bounds.end(); ++it) {
        int *param = stats.ptr<int>(static_cast<int>(it - bounds.begin()));
        param[cv::ConnectedComponentsTypes::CC_STAT_LEFT] = it->at(0);
        param[cv::ConnectedComponentsTypes::CC_STAT_TOP] = it->at(1);
        param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH] = it->at(2) - it->at(0);
        param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT] = it->at(3) - it->at(1);
      }

      return n8x8;
    };

    //////////////////////////////////////////////////
    std::array<cv::Point2f, 4> getCornersInBoundingBox
      (cv::Mat &image, cv::Rect bb)
    {
      std::array<cv::Point2f, 4> corners;

      float dist_tl = std::numeric_limits<float>::max();
      float dist_tr = std::numeric_limits<float>::max();
      float dist_br = std::numeric_limits<float>::max();
      float dist_bl = std::numeric_limits<float>::max();
      for (unsigned int i = 0; i < bb.height; ++i)
        for (unsigned int j = 0; j < bb.width; ++j)
          if (static_cast<int>(image.at<uchar>(bb.y + i, bb.x + j)) == 255) {
            float dist;
            // check for top left
            dist = std::pow(i, 2) + std::pow(j, 2);
            if (dist < dist_tl) {
              corners.at(0) = cv::Point2f(j, i); dist_tl = dist;
            }
            // check for top right
            dist = std::pow(bb.width - j, 2) + std::pow(i, 2);
            if (dist < dist_tr) {
              corners.at(1) = cv::Point2f(j, i); dist_tr = dist;
            }
            // check for bottom right
            dist = std::pow(bb.width - j, 2) + std::pow(bb.height - i, 2);
            if (dist < dist_br) {
              corners.at(2) = cv::Point2f(j, i); dist_br = dist;
            }
            // check for bottom left
            dist = std::pow(j, 2) + std::pow(bb.height - i, 2);
            if (dist < dist_bl) {
              corners.at(3) = cv::Point2f(j, i); dist_bl = dist;
            }
          }

      return corners;
    };

  }
}

#endif
