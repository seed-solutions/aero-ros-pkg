#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Core>
#include <Eigen/SVD>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <tf/transform_broadcaster.h>
#include <aero_startup/PointXYZHSI.h>

/*
  @define srv 1
  float32 x_cap
  float32 y_cap
  float32 z_cap
  float32 x
  float32 y
  float32 z
  int8 h_cap
  uint8 s_cap
  uint8 i_cap
  int8 h
  uint8 s
  uint8 i
  bool precise
  ---
  int8 status
  bool prior_setting
*/

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

int status;

hsi target_hsi_max;
hsi target_hsi_min;
xyz space_min;
xyz space_max;

bool extract_position_only;

// ros::Publisher pcl_pub;

//////////////////////////////////////////////////
bool ValidHSI(rgb _color)
{
  hsi hsi_color;
  // I
  float I_max = std::max({_color.r, _color.g, _color.b});
  hsi_color.i = I_max;
  if (hsi_color.i < target_hsi_min.i || hsi_color.i > target_hsi_max.i)
    return false;
  // S
  float i_min = std::min({_color.r, _color.g, _color.b});
  if (I_max > 0) hsi_color.s = 255 * (1 - i_min / I_max);
  else hsi_color.s = 0;
  if (hsi_color.s < target_hsi_min.s || hsi_color.s > target_hsi_max.s)
    return false;
  // H
  if (static_cast<int>(I_max) == static_cast<int>(i_min))
  {
    hsi_color.h = 0;
  }
  else
  {
    if (_color.g > _color.b)
    {
      if (_color.r > _color.g)
	hsi_color.h = 60 * (_color.g - _color.b) / (I_max - i_min);
      else
	hsi_color.h = 60 * (2 + (_color.b - _color.r) / (I_max - i_min));
    }
    else
    {
      if (_color.r > _color.b)
	hsi_color.h = 360 - 60 * (_color.b - _color.g) / (I_max - i_min);
      else
	hsi_color.h = 60 * (4 + (_color.r - _color.g) / (I_max - i_min));
    }
    if (hsi_color.h <= 180) // 0 ~ 180 -> 0 ~ 127
      hsi_color.h = static_cast<int>(hsi_color.h / 180.0 * 127);
    else // 180 ~ 360 -> -128 ~ 0
      hsi_color.h = static_cast<int>((hsi_color.h - 360) / 180.0 * 128);
  }
  if (hsi_color.h < target_hsi_min.h || hsi_color.h > target_hsi_max.h)
    return false;

  return true;
};

//////////////////////////////////////////////////
bool Reconfigure(aero_startup::PointXYZHSI::Request  &req,
		 aero_startup::PointXYZHSI::Response &res)
{
  target_hsi_max.h = req.h_cap;
  target_hsi_min.h = req.h;
  target_hsi_max.s = req.s_cap;
  target_hsi_min.s = req.h;
  target_hsi_max.i = req.i_cap;
  target_hsi_min.i = req.i;
  space_max.x = req.x_cap;
  space_min.x = req.x;
  space_max.y = req.y_cap;
  space_min.y = req.y;
  space_max.z = req.z_cap;
  space_min.z = req.z;
  res.prior_setting = extract_position_only;
  extract_position_only = !req.precise;
  ros::spinOnce();
  res.status = status;
  return true;
};

//////////////////////////////////////////////////
void SubscribePoints(const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  // Analyze points

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl, *raw);

  Eigen::Vector3f raw_center(0, 0, 0);
  std::vector<Eigen::Vector3f> raw_vertices;
  raw_vertices.reserve(raw->points.size());
  for (unsigned int i = 0; i < raw->points.size(); ++i)
    if ((raw->points[i].x > space_min.x) &&
        (raw->points[i].x < space_max.x) &&
        (raw->points[i].y > space_min.y) &&
        (raw->points[i].y < space_max.y) &&
        (raw->points[i].z > space_min.z) &&
        (raw->points[i].z < space_max.z))
    {
      rgb color = {raw->points[i].r, raw->points[i].g, raw->points[i].b};
      if (ValidHSI(color))
      {
	raw_vertices.push_back(
	    Eigen::Vector3f(raw->points[i].x, raw->points[i].y, raw->points[i].z));
	raw_center +=
	    Eigen::Vector3f(raw->points[i].x, raw->points[i].y, raw->points[i].z);
      }
    }
  raw_vertices.resize(raw_vertices.size());
  raw_center = raw_center * (1.0 / raw_vertices.size());

  // debug messages
  ROS_INFO("found %d points", raw_vertices.size());
  ROS_INFO("hsi : [%d ~ %d, %d ~ %d, %d ~ %d]",
	   target_hsi_min.h, target_hsi_max.h, target_hsi_min.s, target_hsi_max.s,
	   target_hsi_min.i, target_hsi_max.i);
  ROS_INFO("xyz : [%f ~ %f, %f ~ %f, %f ~ %f]",
	   space_min.x, space_max.x, space_min.y, space_max.y,
	   space_min.z, space_max.z);

  Eigen::Vector3f raw_variance(0, 0, 0);
  std::vector<Eigen::Vector3f> raw_variances(raw_vertices.size());
  for (unsigned int i = 0; i < raw_vertices.size(); ++i)
  {
    raw_variances[i] = Eigen::Vector3f(
	 (raw_vertices[i][0] - raw_center[0]) * (raw_vertices[i][0] - raw_center[0]),
	 (raw_vertices[i][1] - raw_center[1]) * (raw_vertices[i][1] - raw_center[1]),
	 (raw_vertices[i][2] - raw_center[2]) * (raw_vertices[i][2] - raw_center[2]));
    raw_variance += raw_variances[i];
  }
  raw_variance = raw_variance * (9.0 / raw_vertices.size()); // cutting 5% of points

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.reserve(raw_vertices.size());

  int vertices_count = 0;
  Eigen::Vector3f center(0, 0, 0);
  std::vector<Eigen::Vector3f> vertices;
  vertices.reserve(raw_vertices.size());
  for (unsigned int i = 0; i < raw_vertices.size(); ++i)
    if ((raw_variances[i][0] < raw_variance[0]) &&
	(raw_variances[i][1] < raw_variance[1]) &&
	(raw_variances[i][2] < raw_variance[2]))
    {
      vertices.push_back(raw_vertices[i]);
      center += raw_vertices[i];

      cloud->points.push_back(pcl::PointXYZ(raw_vertices[i][0],
					    raw_vertices[i][1], raw_vertices[i][2]));
      ++vertices_count;
    }
  vertices.resize(vertices_count);
  cloud->points.resize(vertices_count);
  center = center * (1.0 / vertices_count);

  // Calculate Object Normal of each cloud point

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.01);
  ne.compute(*normals);

  if (normals->points.size() == 0)
  {
    ROS_ERROR("no points detected");
    status = 0;
    return; // object detection failed
  }

  // Return only positions

  if (extract_position_only)
  {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
    tf::Quaternion q(0, 0, 0, 1);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
					  "ps4eye_frame", "object_l"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
					  "ps4eye_frame", "object_r"));
    status = 1;
    return;
  }

  // Calculate Object Normal (average of point normals)

  int normal_count = 0;
  Eigen::Vector3f normal(0, 0, 0);
  for (unsigned int i = 0; i < normals->points.size(); ++i)
  {
    if (normals->points[i].normal_x != normals->points[i].normal_x ||
	normals->points[i].normal_y != normals->points[i].normal_y ||
	normals->points[i].normal_z != normals->points[i].normal_z)
      continue; // reject point with nan value normal
    ++normal_count;
    normal += Eigen::Vector3f(normals->points[i].normal_x,
			      normals->points[i].normal_y,
			      normals->points[i].normal_z);
  }

  if (normal_count == 0)
  {
    ROS_ERROR("no valid normals");
    status = -4;
    return; // normals were not computed correctly
  }

  normal = normal * (1.0 / normal_count);
  normal.normalize();

  Eigen::Vector3f transpose_normal = Eigen::Vector3f(1, 0, 0).cross(normal);
  float theta_normal  = asin(transpose_normal.norm());
  if (theta_normal != theta_normal) // check nan
  {
    ROS_ERROR("error occured while calculation normal");
    status = -3;
    return;
  }
  if ((Eigen::Vector3f(1, 0, 0)).dot(normal) < 0) theta_normal = M_PI - theta_normal;
  transpose_normal.normalize();

  // Calculate Object Axis

  Eigen::MatrixXf m(vertices.size(), 3);
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    m(i, 0) = vertices[i][0] - center[0];
    m(i, 1) = vertices[i][1] - center[1];
    m(i, 2) = vertices[i][2] - center[2];
  }
  Eigen::JacobiSVD<Eigen::MatrixXf> svd((1.0 / vertices.size()) * m.transpose() * m,
  					Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Vector3f axis = svd.matrixU().col(0);
  axis.normalize();

  Eigen::Vector3f transpose_axis = Eigen::Vector3f(1, 0, 0).cross(axis);
  float theta_axis = asin(transpose_axis.norm());
  if (theta_axis != theta_axis) // check nan
  {
    ROS_ERROR("error occured while calculating axis");
    status = -2;
    return;
  }
  if ((Eigen::Vector3f(1, 0, 0)).dot(axis) < 0) theta_axis = M_PI - theta_axis;
  transpose_axis.normalize();

  // Extract Object Normal Component Perpendicular to Object Axis

  Eigen::Quaternionf axis_pose_q(cos(theta_axis / 2),
				 transpose_axis[0] * sin(theta_axis / 2),
				 transpose_axis[1] * sin(theta_axis / 2),
				 transpose_axis[2] * sin(theta_axis / 2));

  Eigen::Vector3f normal_x_axis = normal.cross(axis);
  normal_x_axis.normalize();
  Eigen::Vector3f axis_z = axis_pose_q * Eigen::Vector3f(0, 0, 1);
  axis_z.normalize();
  float theta_z = asin((normal_x_axis.cross(axis_z)).norm()) + M_PI/2;
  if (normal_x_axis.dot(axis_z) < 0) theta_z = 2 * M_PI - theta_z;
  Eigen::Quaternionf transpose_z(cos(theta_z / 2), sin(theta_z / 2), 0, 0);
  Eigen::Quaternionf pose_q = axis_pose_q * transpose_z;

  // Transform Object Pose to Grasping Coordinates

  static tf::TransformListener tf_;

  tf::StampedTransform base_to_eye;
  ros::Time now = ros::Time::now();
  tf_.waitForTransform("leg_base_link", "ps4eye_frame", now, ros::Duration(2.0));

  try
  {
    tf_.lookupTransform("leg_base_link", "ps4eye_frame", now, base_to_eye);
    Eigen::Quaternionf base_to_eye_q =
      Eigen::Quaternionf(base_to_eye.getRotation().w(),
			 base_to_eye.getRotation().x(),
			 base_to_eye.getRotation().y(),
			 base_to_eye.getRotation().z());
    Eigen::Quaternionf world_q = base_to_eye_q * pose_q;
    Eigen::Vector3f world_center =
      Eigen::Vector3f(base_to_eye.getOrigin().x(),
		      base_to_eye.getOrigin().y(),
		      base_to_eye.getOrigin().z()) + base_to_eye_q * center;
    // X : axis, Z : normal -> X : normal , Z : axis
    Eigen::Quaternionf grasp_q = // rotate on axis Y by M_PI/2
      world_q * Eigen::Quaternionf(0.707107, 0.0, 0.707107, 0.0);
    Eigen::Vector3f coordinate_sgn = grasp_q * Eigen::Vector3f(0, 0, 1);
    Eigen::Quaternionf pose_q_l = grasp_q;
    Eigen::Quaternionf pose_q_r = grasp_q;
    // reset coords to grasp-able direction if needed
    if (coordinate_sgn.dot(Eigen::Vector3f(-1, -1, 1)) <
	coordinate_sgn.dot(Eigen::Vector3f(1, 1, -1)))
      pose_q_l = grasp_q * Eigen::Quaternionf(0.0, 1.0, 0.0, 0.0);
    if (coordinate_sgn.dot(Eigen::Vector3f(-1, 1, 1)) <
	coordinate_sgn.dot(Eigen::Vector3f(1, -1, -1)))
      pose_q_r = grasp_q * Eigen::Quaternionf(0.0, 1.0, 0.0, 0.0);

    // feature in left-hand grasp coords
    static tf::TransformBroadcaster br_l;
    tf::Transform transform_l;
    transform_l.setOrigin(
        tf::Vector3(world_center[0], world_center[1], world_center[2]));
    tf::Quaternion q_l(pose_q_l.x(), pose_q_l.y(), pose_q_l.z(), pose_q_l.w());
    transform_l.setRotation(q_l);
    br_l.sendTransform(tf::StampedTransform(transform_l, ros::Time::now(),
					    "leg_base_link", "object_l"));

    // feature in right-hand grasp coords
    static tf::TransformBroadcaster br_r;
    tf::Transform transform_r;
    transform_r.setOrigin(
        tf::Vector3(world_center[0], world_center[1], world_center[2]));
    tf::Quaternion q_r(pose_q_r.x(), pose_q_r.y(), pose_q_r.z(), pose_q_r.w());
    transform_r.setRotation(q_r);
    br_r.sendTransform(tf::StampedTransform(transform_r, ros::Time::now(),
					    "leg_base_link", "object_r"));

    status = 1;
  }
  catch (std::exception e)
  {
    ROS_ERROR("failed tf listen");
    status = -1;
  }

  // Export results (mainly for debug)

  // pcl::PCLPointCloud2 pcl_out;
  // sensor_msgs::PointCloud2 msg;
  // pcl::toPCLPointCloud2(*cloud, pcl_out);
  // pcl_conversions::fromPCL(pcl_out, msg);
  // msg.header.frame_id = "ps4eye_frame";
  // msg.header.stamp = ros::Time(0); // get possible recent
  // pcl_pub.publish(msg);
};


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_features");
  ros::NodeHandle nh;

  status = 0;
  target_hsi_max = {0, 0, 0};
  target_hsi_min = {0, 0, 0};
  space_min = {-0.3, -0.3, 0.0};
  space_max = {0.3, 0.3, 1.0};
  extract_position_only = false;

  ros::ServiceServer service =
      nh.advertiseService("/extract_object_features/perception_area", Reconfigure);
  ros::Subscriber sub = nh.subscribe("/stereo/points2", 1000,
				     SubscribePoints);

  // pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/visualized_object_pcl", 100);

  ros::spin();

  return 0;
}
