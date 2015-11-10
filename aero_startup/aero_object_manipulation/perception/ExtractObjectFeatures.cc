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
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

ros::Publisher markern_pub;
ros::Publisher pcl_pub;

void SubscribePoints(const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  // Analyze points

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl, *raw);

  Eigen::Vector3f raw_center(0, 0, 0);
  std::vector<Eigen::Vector3f> raw_vertices(raw->points.size());
  for (unsigned int i = 0; i < raw->points.size(); ++i)
  {
    raw_vertices[i] =
      Eigen::Vector3f(raw->points[i].x, raw->points[i].y, raw->points[i].z);
    raw_center += raw_vertices[i];
  }
  raw_center = raw_center * (1.0 / raw_vertices.size());

  Eigen::Vector3f raw_variance(0, 0, 0);
  std::vector<Eigen::Vector3f> raw_variances(raw->points.size());
  for (unsigned int i = 0; i < raw_vertices.size(); ++i)
  {
    raw_variances[i] = Eigen::Vector3f(
	 (raw_vertices[i][0] - raw_center[0]) * (raw_vertices[i][0] - raw_center[0]),
	 (raw_vertices[i][1] - raw_center[1]) * (raw_vertices[i][1] - raw_center[1]),
	 (raw_vertices[i][2] - raw_center[2]) * (raw_vertices[i][2] - raw_center[2]));
    raw_variance += raw_variances[i];
  }
  raw_variance = raw_variance * (1.0 / raw_vertices.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.reserve(raw->points.size());

  int vertices_count = 0;
  Eigen::Vector3f center(0, 0, 0);
  std::vector<Eigen::Vector3f> vertices;
  vertices.reserve(raw->points.size());
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

  // Calculate Object Normal

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.01);
  ne.compute(*normals);

  if (normals->points.size() == 0) return; // object detection failed

  // Calculate Object Normal

  Eigen::Vector3f normal(0, 0, 0);
  for (unsigned int i = 0; i < normals->points.size(); ++i)
    normal += Eigen::Vector3f(normals->points[i].normal_x,
			      normals->points[i].normal_y,
			      normals->points[i].normal_z);
  normal = normal * (1.0 / normals->points.size());
  normal.normalize();

  Eigen::Vector3f transpose_normal = Eigen::Vector3f(1, 0, 0).cross(normal);
  float theta_normal  = asin(transpose_normal.norm());
  if (theta_normal != theta_normal) return; // check nan
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
  if (theta_axis != theta_axis) return; // check nan
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
  }
  catch (std::exception e)
  {
    ROS_ERROR("failed tf listen");
  }


  // Export results

  visualization_msgs::Marker marker_normal;
  marker_normal.header.frame_id = "ps4eye_frame";
  marker_normal.header.stamp = ros::Time::now();
  marker_normal.ns = "normal";
  marker_normal.id = 0;
  marker_normal.type = visualization_msgs::Marker::ARROW;
  marker_normal.action = visualization_msgs::Marker::ADD;
  marker_normal.pose.position.x = center[0];
  marker_normal.pose.position.y = center[1];
  marker_normal.pose.position.z = center[2];
  marker_normal.pose.orientation.x = transpose_normal[0] * sin(theta_normal / 2);
  marker_normal.pose.orientation.y = transpose_normal[1] * sin(theta_normal / 2);
  marker_normal.pose.orientation.z = transpose_normal[2] * sin(theta_normal / 2);
  marker_normal.pose.orientation.w = cos(theta_normal / 2);
  marker_normal.scale.x = 0.0001 * vertices.size(); // 0.1
  marker_normal.scale.y = 0.01;
  marker_normal.scale.z = 0.01;
  marker_normal.color.r = 0.0f;
  marker_normal.color.g = 0.0f;
  marker_normal.color.b = 1.0f;
  marker_normal.color.a = 1.0f;
  marker_normal.lifetime = ros::Duration();
  markern_pub.publish(marker_normal);

  pcl::PCLPointCloud2 pcl_out;
  sensor_msgs::PointCloud2 msg;
  pcl::toPCLPointCloud2(*cloud, pcl_out);
  pcl_conversions::fromPCL(pcl_out, msg);
  msg.header.frame_id = "ps4eye_frame";
  msg.header.stamp = ros::Time(0); // get possible recent
  pcl_pub.publish(msg);

  // originally calculated feature
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
  tf::Quaternion q(pose_q.x(), pose_q.y(), pose_q.z(), pose_q.w());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
					"ps4eye_frame", "object"));
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_features");
  ros::NodeHandle nh;

  markern_pub = nh.advertise<visualization_msgs::Marker>(
      "/visualized_object_normal", 100);

  ros::Subscriber sub = nh.subscribe("/stereo/hsi_color_filter/hsi_output", 1000,
				     SubscribePoints);

  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/visualized_object_pcl", 100);

  ros::spin();

  return 0;
}
