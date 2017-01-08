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
       std::string debug_folder="")
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

      if (debug_folder != "") {
        debug_folder += std::to_string(image.cols) + "x" + std::to_string(image.rows);
        cv::imwrite(debug_folder + ".jpg", image);
        cv::imwrite(debug_folder + "stretched.jpg", image_stretchedin);
        cv::imwrite(debug_folder + "thre.jpg", image8x8in);
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
    bool findPackagePattern(cv::Mat &_input, cv::Mat &_stats,
                            std::string _debug_folder="") {
      cv::Mat img;
      cv::resize(_input, img, cv::Size(400, 400));

      // fft img for filtering
      cv::Mat planes[] = {cv::Mat_<float>(img), cv::Mat::zeros(img.size(), CV_32F)};
      cv::Mat dft;
      merge(planes, 2, dft);
      cv::dft(dft, dft);
      cv::split(dft, planes); // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))

      // mask where color in x direction is stable
      // this assumes that between items are horizontally stable in terms of color
      // by eliminating the between, we are able to part each items in pile
      // if parted items are detected, facet is most likely a pile of items
      // note, dft is not shifted, therefore, mask edge region instead of center
      for (unsigned int i = 0; i < 2; ++i) {
        cv::Mat mask1 = planes[i](cv::Rect(0, 0, 30, img.rows));
        mask1.setTo(0.0);
        cv::Mat mask2 = planes[i](cv::Rect(img.cols - 30, 0, 30, img.rows));
        mask2.setTo(0.0);
      }

      // ifft to get image back
      cv::Mat idft;
      cv::Mat iplanes[] =
        {cv::Mat::zeros(img.size(), CV_32F), cv::Mat::zeros(img.size(), CV_32F)};
      cv::merge(planes, 2, dft);
      cv::idft(dft, idft);
      cv::split(idft, iplanes);
      cv::Mat img_back;
      cv::magnitude(iplanes[0], iplanes[1], img_back);

      // float Mat to gray image
      // cut image, as edge ffts have high magnitude independent to image features
      cv::Mat roi = img_back(cv::Rect(10, 10, img.cols - 20, img.rows - 20));
      cv::Mat img2(roi.rows, roi.cols, CV_8U);
      double max;
      double min; // not used
      cv::minMaxLoc(roi, &min, &max);
      for (unsigned int i = 0; i < img2.rows; ++i)
        for (unsigned int j = 0; j < img2.cols; ++j)
          img2.at<uchar>(i, j) = static_cast<int>(roi.at<float>(i, j) / max * 255);

      // stretch image toward y direction
      // this stresses the in-between features
      // this process lessens false negatives and will not add much false positives
      cv::Mat stretch2;
      cv::resize(img2, stretch2, cv::Size(100, 800));

      // otsu binary
      cv::Mat blur;
      cv::GaussianBlur(stretch2, blur, cv::Size(101, 101), 0);
      cv::Mat binary;
      cv::threshold(blur, binary, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

      cv::resize(binary, img2, cv::Size(400, 400));

      // get width of each cluster
      cv::Mat labeled_image;
      cv::Mat centroids;
      int n_lables =
        cv::connectedComponentsWithStats(img2, labeled_image, _stats, centroids);

      // likeliness1 evaluates likeliness of pile
      // likeliness1 = a cluster with a long width is likely an item
      // false negative
      //   1. when cluster count is 2 (hard to distinguish if two is a pile)
      //      (besides, we should not be interested in pile of two)
      //   2. depending on package design (assumption may not be met)
      int likeliness1 = 0;
      for (int k = 1; k < n_lables; ++k) {
        int *param = _stats.ptr<int>(k);
        int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
        int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];
        if (width > (img2.cols >> 1) - 30) ++likeliness1;

        // TODO: should return _stats in original size
        // 400, 400 -> _input.rows, _input.cols
      }

      if (_debug_folder != "") {
        cv::imwrite(_debug_folder + "local1d_facet_raw.jpg", _input);
        cv::imwrite(_debug_folder + "local1d_facet_fft.jpg", img2);
        cv::imwrite(_debug_folder + "local1d_facet.jpg", stretch2);
      }

      if (likeliness1 > 2)
        return true;

      return false;
    };

  }
}

#endif
