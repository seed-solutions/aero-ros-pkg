#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include <grpc/grpc.h>
#include <grpc++/channel.h>
#include <grpc++/client_context.h>
#include <grpc++/create_channel.h>
#include <grpc++/security/credentials.h>
#include "body_skeleton.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

using bodyskeleton::Joint;
using bodyskeleton::Position;
using bodyskeleton::Orientation;
using bodyskeleton::User;
using bodyskeleton::BodySkeleton;

class KinectClient {
public:
  KinectClient(std::shared_ptr<Channel> channel, ros::NodeHandle _nh)
    : stub_(BodySkeleton::NewStub(channel)), nh_(_nh)
  {
    pub_ = nh_.advertise<std_msgs::Int32>("/kinect/person_exists", 1);
    count_ = 0;
    person_lost_ = 100;
  }

  void GetJoints()
  {
    ClientContext context;
    Joint joint;
    User user;
    user.set_id(0);

    std::unique_ptr<ClientReader<Joint> > reader(
	stub_->GetSkeleton(&context, user));

    int found_joints = 0;
    while (reader->Read(&joint))
    {
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(0,0,0));
      transform.setRotation(
      	  tf::Quaternion(joint.orientation().x(),
      			 joint.orientation().y(),
      			 joint.orientation().z(),
      			 joint.orientation().w()));
      if (joint.name().find("hip_") != std::string::npos ||
      	  joint.name().find("knee_") != std::string::npos ||
      	  joint.name().find("foot_") != std::string::npos)
      {
      	tf::Transform tmp_tf(tf::Matrix3x3(0, 0, -1,
      					   0, -1, 0,
      					   -1, 0, 0));
      	transform = transform * tmp_tf;
      }
      else if (joint.name().find("left_shoulder_") != std::string::npos ||
      	       joint.name().find("left_elbow_") != std::string::npos ||
      	       joint.name().find("left_hand_") != std::string::npos)
      {
      	tf::Transform tmp_tf(tf::Matrix3x3(0, -1, 0,
      					   1, 0, 0,
      					   0, 0, 1));
      	transform = transform * tmp_tf;
      }
      else if (joint.name().find("right_shoulder_") != std::string::npos ||
      	       joint.name().find("right_elbow_") != std::string::npos ||
      	       joint.name().find("right_hand_") != std::string::npos)
      {
      	tf::Transform tmp_tf(tf::Matrix3x3(0, 1, 0,
      					   -1, 0, 0,
      					   0, 0, 1));
      	transform = transform * tmp_tf;
      }

      transform.setOrigin(
      	  tf::Vector3(joint.position().x(), joint.position().y(),
      		      joint.position().z()));

      tf::Transform change_frame;
      change_frame.setOrigin(tf::Vector3(0, 0, 0));
      tf::Quaternion frame_rotation;
      frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
      change_frame.setRotation(frame_rotation);
      transform = change_frame * transform;

      transform_[joint.name()] = transform;

      ++found_joints;
      ROS_INFO("read %s %f, %f, %f", joint.name().c_str(),
	       joint.position().x(), joint.position().y(),
	       joint.position().z());
    }

    if (found_joints == 0) ++person_lost_;
    else person_lost_ = 0;

    std_msgs::Int32 msg;
    if (person_lost_ > 30) msg.data = 0;
    else msg.data = 1;

    pub_.publish(msg);

    ROS_INFO("once %d", count_);
    ++count_;
  }

  void Broadcast()
  {
    static tf::TransformBroadcaster br_kinect;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0,0,0));
    transform.setRotation(
			  tf::Quaternion(0, 0, 0, 1));

    br_kinect.sendTransform(
	  tf::StampedTransform(transform, ros::Time::now(),
			       "/leg_base_link", "/kinect"));

    for (auto iter = transform_.begin(); iter != transform_.end(); ++iter)
    {
      static tf::TransformBroadcaster br;

      if (person_lost_ > 30)
	br.sendTransform(
	    tf::StampedTransform(transform, ros::Time::now(),
				 "/kinect", iter->first));
      else
	br.sendTransform(
	    tf::StampedTransform(iter->second, ros::Time::now(),
				 "/kinect", iter->first));
    }
  }

private:

  std::unique_ptr<BodySkeleton::Stub> stub_;

  ros::NodeHandle nh_;

  ros::Publisher pub_;

  int count_;

  int person_lost_;

  std::map<std::string, tf::Transform> transform_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "kinect_client");

  ros::NodeHandle nh;

  KinectClient client(
      grpc::CreateChannel("192.168.101.190:50052", grpc::InsecureCredentials()),
      nh);

  ros::Rate loop_rate(30);

  while(ros::ok)
  {
    client.GetJoints();
    client.Broadcast();
    loop_rate.sleep();
  }
}
