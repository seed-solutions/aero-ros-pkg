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

// ros::Publisher marker_pub;
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

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.01);
  ne.compute(*normals);

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
  if (theta_normal != theta_normal) return; // nan
  if ((Eigen::Vector3f(1, 0, 0)).dot(normal) < 0) theta_normal = M_PI - theta_normal;
  transpose_normal.normalize();


  // vertices.resize(vertices_count * 2);
  // cloud->points.resize(vertices_count * 2);
  // for (unsigned int i = 0; i < vertices_count; ++i)
  // {
  //   vertices[vertices_count + i] =
  //     vertices[i] - 2 * (vertices[i] - center).dot(normal) * normal;
  //   cloud->points[vertices_count + i] =
  //     pcl::PointXYZ(vertices[vertices_count + i][0],
  // 		    vertices[vertices_count + i][1],
  // 		    vertices[vertices_count + i][2]);    
  // }

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

  // std::vector<Eigen::Vector3f> projected_points(vertices.size());
  // for (unsigned int i = 0; i < vertices.size(); ++i)
  //   projected_points[i] =
  //     vertices[i] - (vertices[i] - center).dot(normal) * normal;

  Eigen::Vector3f transpose_axis = Eigen::Vector3f(1, 0, 0).cross(axis);
  float theta_axis = asin(transpose_axis.norm());
  if (theta_axis != theta_axis) return; // nan
  if ((Eigen::Vector3f(1, 0, 0)).dot(axis) < 0) theta_axis = M_PI - theta_axis;
  transpose_axis.normalize();


  // Debug

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

  // visualization_msgs::Marker marker_axis;
  // marker_axis.header.frame_id = "ps4eye_frame";
  // marker_axis.header.stamp = ros::Time::now();
  // marker_axis.ns = "object";
  // marker_axis.id = 0;
  // marker_axis.type = visualization_msgs::Marker::ARROW;
  // marker_axis.action = visualization_msgs::Marker::ADD;
  // marker_axis.pose.position.x = center[0];
  // marker_axis.pose.position.y = center[1];
  // marker_axis.pose.position.z = center[2];
  // marker_axis.pose.orientation.x = axis_pose_q.x();
  // marker_axis.pose.orientation.y = axis_pose_q.y();
  // marker_axis.pose.orientation.z = axis_pose_q.z();
  // marker_axis.pose.orientation.w = axis_pose_q.w();
  // marker_axis.scale.x = 0.0001 * vertices.size(); // 0.1
  // marker_axis.scale.y = 0.01;
  // marker_axis.scale.z = 0.01;
  // marker_axis.color.r = 0.0f;
  // marker_axis.color.g = 1.0f;
  // marker_axis.color.b = 0.0f;
  // marker_axis.color.a = 1.0f;
  // marker_axis.lifetime = ros::Duration();
  // marker_pub.publish(marker_axis);

  pcl::PCLPointCloud2 pcl_out;
  sensor_msgs::PointCloud2 msg;
  pcl::toPCLPointCloud2(*cloud, pcl_out);
  pcl_conversions::fromPCL(pcl_out, msg);
  msg.header.frame_id = "ps4eye_frame";
  msg.header.stamp = ros::Time(0); // get possible recent
  pcl_pub.publish(msg);

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
  tf::Quaternion q(pose_q.x(), pose_q.y(), pose_q.z(), pose_q.w());
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
					"ps4eye_frame", "object"));

  // blue : -107 ~ -128 , 235 ~ 255
  // red : 0 ~ 17 , 235 ~ 255
  // green : 46 ~ 97, 42 ~ 194 //56 ~ 49 , 106 ~ 183
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_features");
  ros::NodeHandle nh;

  // marker_pub = nh.advertise<visualization_msgs::Marker>("/visualized_object_feature", 100);
  markern_pub = nh.advertise<visualization_msgs::Marker>("/visualized_object_normal", 100);
  ros::Subscriber sub = nh.subscribe("/stereo/hsi_color_filter/hsi_output", 1000,
				     SubscribePoints);

  pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/visualized_object_pcl", 100);

  ros::spin();

  return 0;
}
