#include <ros/ros.h>
#include "aero_sensors/XtionInterface.hh"

#include <opencv2/core/utility.hpp>
#include <opencv2/saliency.hpp>
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
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_region.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_bounds_sample");
  ros::NodeHandle nh;

  xtion::interface::XtionInterfacePtr xtion
    (new xtion::interface::XtionInterface(nh));

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

  // plane segmentation
  pcl::OrganizedMultiPlaneSegmentation<
    pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
  mps.setMinInliers(static_cast<int>(1000 * resize_x * resize_y));
  mps.setAngularThreshold(0.034906);
  mps.setDistanceThreshold(0.02);
  mps.setInputNormals(normal);
  mps.setInputCloud(cloud);
  std::vector
    <pcl::PlanarRegion<pcl::PointXYZRGB>,
     Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > >
    regions;
  mps.segmentAndRefine(regions);

  // find nearest plane region
  float min_dist = std::numeric_limits<int>::max();
  int target_idx = -1;
  for (unsigned int i = 0; i < regions.size(); ++i) {
    Eigen::Vector3f centroid = regions[i].getCentroid();
    float dist = std::pow(centroid.x(), 2) + std::pow(centroid.y(), 2)
      + std::pow(centroid.z(), 2);
    if (dist < min_dist) {
      min_dist = dist;
      target_idx = i;
    }
  }

  // get plane info
  Eigen::Vector4f plane = regions[target_idx].getCoefficients();
  Eigen::Vector3f plane_center = regions[target_idx].getCentroid();
  float plane_size = -1;
  for (auto it = regions[target_idx].getContour().begin();
       it != regions[target_idx].getContour().end(); ++it) {
    float size = std::pow(it->x - plane_center.x(), 2)
      + std::pow(it->y - plane_center.y(), 2)
      + std::pow(it->z - plane_center.z(), 2);
    if (size > plane_size)
      plane_size = size;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_above_plane
    (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> raw_indices;
  points_above_plane->points.reserve(cloud->points.size());
  raw_indices.reserve(cloud->points.size());

  float plane_min_x = std::numeric_limits<int>::max();
  float plane_min_y = std::numeric_limits<int>::max();
  float plane_max_x = std::numeric_limits<int>::min();
  float plane_max_y = std::numeric_limits<int>::min();
  int plane_min_x_idx = -1, plane_min_y_idx = -1;
  int plane_max_x_idx = -1, plane_max_y_idx = -1;

  // get points on plane
  for (auto it = cloud->points.begin(); it != cloud->points.end(); ++it) {
    if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z))
      continue; // reject nan value

    if (it->x * plane[0] + it->y * plane[1] + it->z * plane[2] + plane[3] > -0.01)
      continue; // points are under plane

    if (std::pow(it->x - plane_center.x(), 2)
        + std::pow(it->y - plane_center.y(), 2)
        + std::pow(it->z - plane_center.z(), 2) >= plane_size)
      continue;

    // get x min and max on plane
    if (it->x < plane_min_x) {
      plane_min_x = it->x;
      plane_min_x_idx = static_cast<int>(it - cloud->points.begin());
    } else if (it->x > plane_max_x) {
      plane_max_x = it->x;
      plane_max_x_idx = static_cast<int>(it - cloud->points.begin());
    }

    // get y min and max on plane
    if (it->y < plane_min_y) {
      plane_min_y = it->y;
      plane_min_y_idx = static_cast<int>(it - cloud->points.begin());
    } else if (it->y > plane_max_y) {
      plane_max_y = it->y;
      plane_max_y_idx = static_cast<int>(it - cloud->points.begin());
    }

    points_above_plane->points.push_back(*it);
    raw_indices.push_back(static_cast<int>(it - cloud->points.begin()));
  }

  // euclidean cluster points on plane
  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(static_cast<int>(200 * resize_x * resize_y));
  ec.setMaxClusterSize(25000);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree
    (new pcl::search::KdTree<pcl::PointXYZRGB>);
  kdtree->setInputCloud(points_above_plane);
  ec.setSearchMethod(kdtree);
  ec.setInputCloud(points_above_plane);
  ec.extract(clusters);

  // find the bounds of each cluster (bound depth indicies)
  // note: vision is always xy plane, so bounds are also xy based
  std::vector<std::array<int, 4> > cluster_bounds;
  cluster_bounds.reserve(clusters.size() + 1);
  cluster_bounds.push_back({plane_min_x_idx, plane_min_y_idx, plane_max_x_idx, plane_max_y_idx});
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
        {points_above_plane->points[*pit].x, points_above_plane->points[*pit].y,
         points_above_plane->points[*pit].z};

      if (this_p.x() < min_x) {
        min_x = this_p.x();
        min_x_idx = raw_indices[*pit];
      } else if (this_p.x() > max_x) {
        max_x = this_p.x();
        max_x_idx = raw_indices[*pit];
      }

      if (this_p.y() < min_y) {
        min_y = this_p.y();
        min_y_idx = raw_indices[*pit];
      } else if (this_p.y() > max_y) {
        max_y = this_p.y();
        max_y_idx = raw_indices[*pit];
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
  cv::waitKey(100000);
}
