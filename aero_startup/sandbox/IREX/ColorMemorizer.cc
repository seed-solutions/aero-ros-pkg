#include <ros/ros.h>
#include <math.h>
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <std_msgs/Int32.h>

static const int NUM_OF_PALETTES = 8; // number of dominant colors to find
static const int NUM_OF_LAYERS = 3; // log2 NUM_OF_PALETTES
// number of colors in palette that should match the object color
static const int VALID_MATCH = 4;
// only look for objects with more than this many points
static const int MINIMUM_POINTS = 100;

ros::Publisher robot_pose_pub;
ros::Subscriber joint_subscriber;
ros::Subscriber cloud_subscriber;
ros::Subscriber person_subscriber;

bool in_wait;
bool in_check;

struct rgb
{
  int r;
  int g;
  int b;
};

struct hsi
{
  std::string label;
  int h_max;
  int h_min;
  int s_max;
  int s_min;
  int i_max;
  int i_min;
};

std::vector<hsi> samples = {
  {"RED",  17, 0,  255, 235,  255, 88},
  {"GREEN",  97, 46,  194, 42,  183, 106},
  {"BLUE",  -107, -128,  255, 235,  255, 120}
};


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
void SubscribePoints(const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  if (!in_wait) return;

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl, *raw);

  Eigen::Vector3f space_min(-0.5, -0.5, -0.5);
  Eigen::Vector3f space_max(0.5, 0.5, 0.5);

  // Find points in search space

  std::vector<rgb> extracted;
  extracted.reserve(raw->points.size());
  for (unsigned int i = 0; i < raw->points.size(); ++i)
    if ((raw->points[i].x > space_min[0]) &&
        (raw->points[i].x < space_max[0]) &&
        (raw->points[i].y > space_min[1]) &&
        (raw->points[i].y < space_max[1]) &&
        (raw->points[i].z > space_min[2]) &&
        (raw->points[i].z < space_max[2]))
    {
      rgb tmp = {raw->points[i].r, raw->points[i].g, raw->points[i].b};
      extracted.push_back(tmp);
    }
  extracted.resize(extracted.size());

  if (extracted.size() < MINIMUM_POINTS)
  {
    ROS_WARN("not seeing any object! %d points detected", extracted.size());
    return; // not enough points
  }

  // Median cut points

  std::vector<rgb> palettes(NUM_OF_PALETTES);
  MedianCut(extracted, 0, 0, palettes);

  // Check if detected object matches a known object

  std::vector<int> scores(samples.size(), 0);
  for (unsigned int i = 0; i < NUM_OF_PALETTES; ++i)
  {
    ROS_INFO("RGB: %d, %d, %d",
	     palettes[i].r, palettes[i].g, palettes[i].b);

    rgb color = {palettes[i].r, palettes[i].g, palettes[i].b};
    float I_max = std::max({color.r, color.g, color.b});
    float i_min = std::min({color.r, color.g, color.b});
    hsi hsi_color;
    // I
    hsi_color.i_max = I_max;
    // S
    if (I_max > 0) hsi_color.s_max = 255 * (1 - i_min / I_max);
    else hsi_color.s_max = 0;
    // H
    if (static_cast<int>(I_max) == static_cast<int>(i_min))
    {
      hsi_color.h_max = 0;
    }
    else
    {
      if (color.g > color.b)
      {
	if (color.r > color.g)
	  hsi_color.h_max = 60 * (color.g - color.b) / (I_max - i_min);
	else
	  hsi_color.h_max = 60 * (2 + (color.b - color.r) / (I_max - i_min));
      }
      else
      {
	if (color.r > color.b)
	  hsi_color.h_max = 360 - 60 * (color.b - color.g) / (I_max - i_min);
	else
	  hsi_color.h_max = 60 * (4 + (color.r - color.g) / (I_max - i_min));
      }
      if (hsi_color.h_max <= 180) // 0 ~ 180 -> 0 ~ 127
	hsi_color.h_max =
	  static_cast<int>(hsi_color.h_max / 180.0 * 127);
      else
	hsi_color.h_max = // 180 ~ 360 -> -128 ~ 0
	  static_cast<int>((hsi_color.h_max - 360) / 180.0 * 128);
    }

    ROS_INFO("HSI: %d, %d, %d",
	     hsi_color.h_max, hsi_color.s_max, hsi_color.i_max);

    for (unsigned int j = 0; j < samples.size(); ++j)
    {
      if (hsi_color.h_max >= samples[j].h_min &&
	  hsi_color.h_max <= samples[j].h_max &&
	  hsi_color.s_max >= samples[j].s_min &&
	  hsi_color.s_max <= samples[j].s_max &&
	  hsi_color.i_max >= samples[j].i_min &&
	  hsi_color.i_max <= samples[j].i_max)
	++scores[j];
    }
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
    return; // not a valid match
  }

  hsi target = samples[most_likely_match];
  ROS_INFO("recognized %s", target.label.c_str());

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = {"neck_y_joint", "neck_p_joint"};
  msg.points.resize(1);
  msg.points[0].positions = {0.0, 0.698};
  robot_pose_pub.publish(msg);

  usleep(static_cast<int32_t>(2 * 1000 * 1000));

  dynamic_reconfigure::ReconfigureRequest req;
  dynamic_reconfigure::ReconfigureResponse res;
  dynamic_reconfigure::Config conf;
  conf.ints.reserve(6);
  std::vector<std::string> param_names =
    {"h_limit_max", "h_limit_min", "s_limit_max", "s_limit_min",
     "i_limit_max", "i_limit_min"};
  std::vector<int> param_values =
    {target.h_max, target.h_min, target.s_max, target.s_min,
     target.i_max, target.i_min};
  for (unsigned int i = 0; i < 6; ++i)
  {
    dynamic_reconfigure::IntParameter tmp;
    tmp.name = param_names[i];
    tmp.value = param_values[i];
    conf.ints.push_back(tmp);
  }
  req.config = conf;
  ros::service::call("/stereo/hsi_color_filter/hsi_filter/set_parameters", req, res);

  in_wait = false;
  in_check = true;
};

//////////////////////////////////////////////////
void SubscribeRobotPose
(const pr2_controllers_msgs::JointTrajectoryControllerState _msg)
{
  // if (!in_check) return;

  // if (fabs(_msg.actual.positions[14]) > 0.78 && // neck_y
  //     fabs(_msg.actual.positions[15]) < 0.01) // neck_p
  // {
  //   in_wait = true;
  //   in_check = false;
  // }
};

//////////////////////////////////////////////////
void SubscribePersonExists(const std_msgs::Int32 _msg)
{
  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = {"neck_y_joint", "neck_p_joint"};
  msg.points.resize(1);

  if (_msg.data == 1 && !in_wait)
  {
    msg.points[0].positions = {-0.79, 0.0};
    robot_pose_pub.publish(msg);
    usleep(static_cast<int32_t>(2 * 1000 * 1000));
    in_wait = true;
  }
  else if (_msg.data == 0 && in_wait)
  {
    msg.points[0].positions = {0.0, 0.0};
    robot_pose_pub.publish(msg);
    usleep(static_cast<int32_t>(2 * 1000 * 1000));    
    in_wait = false;
  }
};

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "color_memorizer");
  ros::NodeHandle nh;

  in_wait = false;
  in_check = true;
  joint_subscriber = nh.subscribe("/aero_controller/state",
				  1, SubscribeRobotPose);
  cloud_subscriber = nh.subscribe("/stereo/points2",
				  1000, SubscribePoints);
  person_subscriber = nh.subscribe("/kinect/person_exists",
				   1, SubscribePersonExists);
  robot_pose_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/aero_controller/command", 100);

  ros::spin();

  return 0;
}
