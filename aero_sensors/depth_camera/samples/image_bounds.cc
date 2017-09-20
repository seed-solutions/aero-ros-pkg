/// @brief sample for boundary estimation
/// @author Kazuhiro Sasabuchi

#include <ros/ros.h>
#include "aero_sensors/DepthCameraInterface.hh"

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/region_growing.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_bounds_sample");
  ros::NodeHandle nh;

  depth_camera::interface::DepthCameraInterfacePtr xtion
    (new depth_camera::interface::DepthCameraInterface(nh));

  float resize_x = 0.25;
  float resize_y = 0.25;

  auto points = xtion->ReadPoints(resize_x, resize_y);

  // ros msg -> pcl PointCloud
  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(points, pcl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl, *cloud);

  // get normal
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree
    (new pcl::search::KdTree<pcl::PointXYZRGB>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);
  pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normal);

  // cluster with region growing
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud);
  reg.setInputNormals(normal);
  reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  // find the bounds of each cluster (bound depth indicies)
  std::vector<std::array<int, 4> > cluster_bounds;
  cluster_bounds.reserve(clusters.size());
  for (auto it = clusters.begin(); it != clusters.end(); ++it) {
    float min_x = std::numeric_limits<int>::max();
    float min_y = std::numeric_limits<int>::max();
    float max_x = std::numeric_limits<int>::min();
    float max_y = std::numeric_limits<int>::min();
    int min_x_idx = -1, min_y_idx = -1;
    int max_x_idx = -1, max_y_idx = -1;

    // for each point in cluster
    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      Eigen::Vector3f this_p =
        {cloud->points[*pit].x, cloud->points[*pit].y, cloud->points[*pit].z};

      if (this_p.x() < min_x) {
        min_x = this_p.x();
        min_x_idx = *pit;
      } else if (this_p.x() > max_x) {
        max_x = this_p.x();
        max_x_idx = *pit;
      }

      if (this_p.y() < min_y) {
        min_y = this_p.y();
        min_y_idx = *pit;
      } else if (this_p.y() > max_y) {
        max_y = this_p.y();
        max_y_idx = *pit;
      }
    }

    cluster_bounds.push_back({min_x_idx, min_y_idx, max_x_idx, max_y_idx});
  }

  // get 2d image bounds from 3d indices
  auto bounds = xtion->ImageBounds(cluster_bounds, resize_x, resize_y);

  // get image as cv::Mat
  auto image = xtion->ReadImage();
  cv::Mat img(image.height, image.width, CV_8UC3);
  int k = 0;
  for (unsigned int i = 0; i < img.rows; ++i) {
    for (unsigned int j = 0; j < img.cols; ++j) {
      img.at<cv::Vec3b>(i, j) =
        cv::Vec3b(image.data[k++], image.data[k++], image.data[k++]);
    }
  }

  // draw box around found bounds
  for (auto it = bounds.begin(); it != bounds.end(); ++it)
    cv::rectangle(img, cv::Rect(it->x_offset, it->y_offset, it->width, it->height),
                  cv::Scalar(0, 255, 0), 2);

  // show results
  cv::namedWindow("bounds", CV_WINDOW_NORMAL);
  cv::resizeWindow("bounds", 640, 480);
  cv::imshow("bounds", img);
  cv::waitKey(10000);
}
