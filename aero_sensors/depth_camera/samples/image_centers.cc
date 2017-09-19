#include <ros/ros.h>
#include "aero_sensors/DepthCameraInterface.hh"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/RegionOfInterest.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/saliency.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_centers_sample");
  ros::NodeHandle nh;

  depth_camera::interface::DepthCameraInterfacePtr xtion
    (new depth_camera::interface::DepthCameraInterface(nh));

  auto image = xtion->ReadImage();

  // ros msg -> cv::Mat
  cv::Mat img(image.height, image.width, CV_8UC3);
  int k = 0;
  for (unsigned int i = 0; i < img.rows; ++i) {
    for (unsigned int j = 0; j < img.cols; ++j) {
      img.at<cv::Vec3b>(i, j) =
        cv::Vec3b(image.data[k++], image.data[k++], image.data[k++]);
    }
  }

  // compute saliency
  auto saliency_algorithm = cv::saliency::Saliency::create("SPECTRAL_RESIDUAL");

  cv::Mat saliency_map;
  if (!saliency_algorithm->computeSaliency(img, saliency_map)) {
    ROS_ERROR("failed saliency");
    return -1;
  }

  cv::Mat binary_map;
  cv::saliency::StaticSaliencySpectralResidual spec;
  spec.computeBinaryMap(saliency_map, binary_map);

  // get saliency bounds
  cv::Mat labeled_image;
  cv::Mat stats;
  cv::Mat centroids;
  int n_labels = cv::connectedComponentsWithStats(binary_map, labeled_image, stats, centroids);

  std::vector<sensor_msgs::RegionOfInterest> bounds;
  bounds.reserve(n_labels);
  for (int i = 1; i < n_labels; ++i) {
    int *param = stats.ptr<int>(i);

    int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
    int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
    int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
    int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

    sensor_msgs::RegionOfInterest roi;
    roi.x_offset = x;
    roi.y_offset = y;
    roi.width = width;
    roi.height = height;
    bounds.push_back(roi);

    cv::rectangle(img, cv::Rect(x, y, width, height), cv::Scalar(0, 255, 0), 2);
  }

  // get 3d centers from 2d image bounds
  auto centers = xtion->ImageCenters(bounds);

  for (int i = 0; i < centers.size(); ++i) {
    auto c = centers.begin() + i;
    auto b = bounds.begin() + i;
    cv::putText(img, std::to_string(c->x) + ", " + std::to_string(c->y) + ", " + std::to_string(c->z),
                cv::Point(b->x_offset, b->y_offset + b->height),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1.0);
  }

  // show results
  cv::namedWindow("bounds", CV_WINDOW_NORMAL);
  cv::resizeWindow("bounds", 1920, 1080);
  cv::imshow("bounds", img);
  cv::waitKey(10000);
}
