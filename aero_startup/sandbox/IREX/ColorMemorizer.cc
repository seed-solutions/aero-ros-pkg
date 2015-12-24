#include <ros/ros.h>
#include <math.h>
#include <chrono>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <aero_startup/PointXYZ.h>
#include <aero_startup/PointHSIArray.h>

/*
  @define srv 1
  float32 x_cap
  float32 y_cap
  float32 z_cap
  float32 x
  float32 y
  float32 z
  ---
  int8 status
*/

/*
  @define srv 2
  int8 size
  int8[] h_cap_v
  uint8[] s_cap_v
  uint8[] i_cap_v
  int8[] h_v
  uint8[] s_v
  uint8[] i_v
  float32 time
  ---
  int8 status
*/

static const int NUM_OF_PALETTES = 8; // number of dominant colors to find
static const int NUM_OF_LAYERS = 3; // log2 NUM_OF_PALETTES
// number of colors in palette that should match the object color
static const int VALID_MATCH = 1;
// only look for objects with more than this many points
static const int MINIMUM_POINTS = 100;

ros::Subscriber cloud_subscriber;

struct rgb
{
  int r;
  int g;
  int b;
};

struct hsi
{
  int h;
  int s;
  int i;
};

struct xyz
{
  float x;
  float y;
  float z;
};

typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;

std::vector<hsi> samples_hsi_max;
std::vector<hsi> samples_hsi_min;

xyz space_min;
xyz space_max;

bool idle;
int result;

//////////////////////////////////////////////////
void MedianCut(std::vector<rgb> _points, int _bucket_idx, int _layer,
	       std::vector<rgb> &_palettes)
{
  rgb max = {0, 0, 0};
  rgb min = {255, 255, 255};

  for (unsigned int i = 0; i < _points.size(); ++i)
  {
    if (_points[i].r > max.r) max.r = _points[i].r;
    if (_points[i].r < min.r) min.r = _points[i].r;
    if (_points[i].g > max.g) max.g = _points[i].g;
    if (_points[i].g < min.g) min.g = _points[i].g;
    if (_points[i].b > max.b) max.b = _points[i].b;
    if (_points[i].b < min.b) min.b = _points[i].b;
  }

  rgb range = {max.r - min.r, max.g - min.g, max.b - min.b};
  std::function<bool(rgb, rgb)> compare;

  if (range.r > range.g)
  {
    if (range.r > range.b)
      compare = [=](rgb a, rgb b){return a.r > b.r;};
    else
      compare = [=](rgb a, rgb b){return a.b > b.b;};
  }
  else
  {
    if (range.g > range.b)
      compare = [=](rgb a, rgb b){return a.g > b.g;};
    else
      compare = [=](rgb a, rgb b){return a.b > b.b;};
  }

  std::sort(_points.begin(), _points.end(), compare);
  int mid = _points.size() * 0.5;

  std::vector<rgb> bucket_one(_points.begin(), _points.begin() + mid);
  std::vector<rgb> bucket_two(_points.begin() + mid, _points.end());

  ++_layer;
  if (_layer >= NUM_OF_LAYERS)
  {
    rgb average_rgb_one = {0, 0, 0};
    for (unsigned int i = 0; i < bucket_one.size(); ++i)
    {
      average_rgb_one.r += bucket_one[i].r;
      average_rgb_one.g += bucket_one[i].g;
      average_rgb_one.b += bucket_one[i].b;
    }
    average_rgb_one.r /= bucket_one.size();
    average_rgb_one.g /= bucket_one.size();
    average_rgb_one.b /= bucket_one.size();
    _palettes[_bucket_idx] = average_rgb_one;

    rgb average_rgb_two = {0, 0, 0};
    for (unsigned int i = 0; i < bucket_two.size(); ++i)
    {
      average_rgb_two.r += bucket_two[i].r;
      average_rgb_two.g += bucket_two[i].g;
      average_rgb_two.b += bucket_two[i].b;
    }
    average_rgb_two.r /= bucket_two.size();
    average_rgb_two.g /= bucket_two.size();
    average_rgb_two.b /= bucket_two.size();
    _palettes[_bucket_idx + 1] = average_rgb_two;

    return;
  }

  MedianCut(bucket_one, _bucket_idx, _layer, _palettes);
  MedianCut(bucket_two, _bucket_idx + (NUM_OF_PALETTES)/(2*_layer), _layer,
	    _palettes);
};

//////////////////////////////////////////////////
bool Request(aero_startup::PointHSIArray::Request  &req,
	     aero_startup::PointHSIArray::Response &res)
{
  samples_hsi_max.clear();
  samples_hsi_min.clear();

  samples_hsi_max.resize(req.size);
  samples_hsi_min.resize(req.size);

  for (unsigned int i = 0; i < req.size; ++i)
  {
    samples_hsi_max[i] = {req.h_cap_v[i], req.s_cap_v[i], req.i_cap_v[i]};
    samples_hsi_min[i] = {req.h_v[i], req.s_v[i], req.i_v[i]};
  }

  idle = false;
  result = -1;
  Clock::time_point start = Clock::now();
  while (result < 0)
  {
    ros::spinOnce();
    Clock::time_point now = Clock::now();
    if (std::chrono::duration_cast<milliseconds>(now - start).count()
	> (req.time * 1000))
      break;
  }
  idle = true;

  res.status = result;
  return true;
};

//////////////////////////////////////////////////
bool Reconfigure(aero_startup::PointXYZ::Request  &req,
		 aero_startup::PointXYZ::Response &res)
{
  space_min.x = req.x;
  space_min.y = req.y;
  space_min.z = req.z;
  space_max.x = req.x_cap;
  space_max.y = req.y_cap;
  space_max.z = req.z_cap;
  res.status = 1;
  return true;
};

//////////////////////////////////////////////////
void SubscribePoints(const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  if (idle) return;

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl, *raw);

  // Find points in search space

  std::vector<rgb> extracted;
  extracted.reserve(raw->points.size());
  for (unsigned int i = 0; i < raw->points.size(); ++i)
    if ((raw->points[i].x > space_min.x) &&
        (raw->points[i].x < space_max.x) &&
        (raw->points[i].y > space_min.y) &&
        (raw->points[i].y < space_max.y) &&
        (raw->points[i].z > space_min.z) &&
        (raw->points[i].z < space_max.z))
    {
      rgb tmp = {raw->points[i].r, raw->points[i].g, raw->points[i].b};
      // get rid of bad hue
      float I_max = std::max({tmp.r, tmp.g, tmp.b});
      float i_min = std::min({tmp.r, tmp.g, tmp.b});
      hsi hsi_color;
      if (I_max > 0) hsi_color.s = 255 * (1 - i_min / I_max);
      else hsi_color.s = 0;
      if (hsi_color.s >= 80) extracted.push_back(tmp);
    }
  extracted.resize(extracted.size());

  if (extracted.size() < MINIMUM_POINTS)
  {
    ROS_WARN("not seeing any object! %d points detected", extracted.size());
    result = -1;
    return; // not enough points
  }

  // Median cut points

  std::vector<rgb> palettes(NUM_OF_PALETTES);
  MedianCut(extracted, 0, 0, palettes);

  // Check if detected object matches a known object

  std::vector<int> scores(samples_hsi_max.size(), 0);
  for (unsigned int i = 0; i < NUM_OF_PALETTES; ++i)
  {
    ROS_INFO("RGB: %d, %d, %d", palettes[i].r, palettes[i].g, palettes[i].b);

    rgb color = {palettes[i].r, palettes[i].g, palettes[i].b};
    float I_max = std::max({color.r, color.g, color.b});
    float i_min = std::min({color.r, color.g, color.b});
    hsi hsi_color;
    // I
    hsi_color.i = I_max;
    // S
    if (I_max > 0) hsi_color.s = 255 * (1 - i_min / I_max);
    else hsi_color.s = 0;
    // H
    if (static_cast<int>(I_max) == static_cast<int>(i_min))
    {
      hsi_color.h = 0;
    }
    else
    {
      if (color.g > color.b)
      {
	if (color.r > color.g)
	  hsi_color.h = 60 * (color.g - color.b) / (I_max - i_min);
	else
	  hsi_color.h = 60 * (2 + (color.b - color.r) / (I_max - i_min));
      }
      else
      {
	if (color.r > color.b)
	  hsi_color.h = 360 - 60 * (color.b - color.g) / (I_max - i_min);
	else
	  hsi_color.h = 60 * (4 + (color.r - color.g) / (I_max - i_min));
      }
      if (hsi_color.h <= 180) // 0 ~ 180 -> 0 ~ 127
	hsi_color.h =
	  static_cast<int>(hsi_color.h / 180.0 * 127);
      else
	hsi_color.h = // 180 ~ 360 -> -128 ~ 0
	  static_cast<int>((hsi_color.h - 360) / 180.0 * 128);
    }

    ROS_INFO("HSI: %d, %d, %d", hsi_color.h, hsi_color.s, hsi_color.i);

    for (unsigned int j = 0; j < samples_hsi_max.size(); ++j)
      if (hsi_color.h >= samples_hsi_min[j].h &&
          hsi_color.h <= samples_hsi_max[j].h &&
          hsi_color.s >= samples_hsi_min[j].s &&
          hsi_color.s <= samples_hsi_max[j].s &&
          hsi_color.i >= samples_hsi_min[j].i &&
          hsi_color.i <= samples_hsi_max[j].i)
	++scores[j];

  }

  int most_likely_match;
  int matched_nums = 0;
  for (unsigned int j = 0; j < scores.size(); ++j)
    if (scores[j] > matched_nums)
    {
      most_likely_match = j;
      matched_nums = scores[j];
    }

  if (matched_nums < VALID_MATCH)
  {
    ROS_WARN("cannot recognize! %d matches not enough", matched_nums);
    result = -1;
    return; // not a valid match
  }

  result = most_likely_match;
};

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_memorizer");
  ros::NodeHandle nh;

  space_min = {-0.5, -0.1, 0.0};
  space_max = {0.5, 0.1, 0.5};
  idle = true;
  result = -1;

  cloud_subscriber = nh.subscribe("/stereo/points2",
				  1000, SubscribePoints);

  ros::ServiceServer service_pa =
    nh.advertiseService("/color_memorizer/perception_area", Reconfigure);
  ros::ServiceServer service_cm =
    nh.advertiseService("/color_memorizer/find_match", Request);

  ros::spin();

  return 0;
}
