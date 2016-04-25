#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include "AeroJointStatePublisher.hh"

using namespace aero;
using namespace controller_manager;

//////////////////////////////////////////////////
AeroJointStatePublisher::AeroJointStatePublisher(ros::NodeHandle _nh)
{
  nh_ = _nh;
  joint_state_publisher_ =
      nh_.advertise<sensor_msgs::JointState>("aero_joint_states", 1);
  stroke_subscriber_ =
      nh_.subscribe("/aero_controller/state",
		    1, &AeroJointStatePublisher::JointStateCallback, this);
  this->Init();
}

//////////////////////////////////////////////////
AeroJointStatePublisher::~AeroJointStatePublisher()
{
}

//////////////////////////////////////////////////
void AeroJointStatePublisher::Init()
{
}

//////////////////////////////////////////////////
void AeroJointStatePublisher::JointStateCallback
(const pr2_controllers_msgs::JointTrajectoryControllerState _msg)
{
}

//////////////////////////////////////////////////
void AeroJointStatePublisher::Main()
{
  joint_state_.header.stamp = ros::Time::now();
  joint_state_publisher_.publish(joint_state_);
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_state_publisher_node");
  ros::NodeHandle nh;

  AeroJointStatePublisherPtr aero;
  aero.reset(new AeroJointStatePublisher(nh));

  ros::Rate loopRate(30);
  while (ros::ok())
  {
    aero->Main();
    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
