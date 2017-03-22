#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32.h"

float angle = 0.0;

void SubscribeAngle(const std_msgs::Float32::ConstPtr& _angle)
{
  angle = -_angle->data * 0.017453292519943295;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_kinect_tf_broadcaster");
  ros::NodeHandle nh;

  ros::Subscriber angle_subscriber =
    nh.subscribe("/kinect_controller/state", 100, &SubscribeAngle);

  float theta0 = -atan2(0.095, 0.164);
  float radius = sqrt(std::pow(0.164, 2) + std::pow(0.095, 2));

  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Rate rate(10.0);

  // base to eye
  ros::Publisher pub =
      nh.advertise<geometry_msgs::Pose>("/matrix/base_to_eye", 1000);

  while (nh.ok())
  {
    ros::spinOnce();
    transform.setOrigin(tf::Vector3(
	radius * cos(angle + theta0), radius * sin(angle + theta0), 0.082));
    transform.setRotation(
        tf::Quaternion(1.570796, 0, 1.570796) * tf::Quaternion(angle, 0, 0));

    br.sendTransform(tf::StampedTransform(
	transform, ros::Time::now(), "base_link", "dynamic_kinect_frame"));

    // base to eye
    geometry_msgs::Pose msg;
    msg.position.x = transform.getOrigin().x();
    msg.position.y = transform.getOrigin().y();
    msg.position.z = transform.getOrigin().z();
    msg.orientation.x = transform.getRotation().w();
    msg.orientation.y = transform.getRotation().x();
    msg.orientation.z = transform.getRotation().y();
    msg.orientation.w = transform.getRotation().z();
    pub.publish(msg);

    rate.sleep();
  }

  return 0;
};
