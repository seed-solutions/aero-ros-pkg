/*
 * This file is modified with a script. Do not Edit!
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

#include "Stroke2Angle.hh"
#include "AeroJointStatePublisher.hh"

namespace aero
{
  namespace world {
    //////////////////////////////////////////////////
    AeroJointStatePublisher::AeroJointStatePublisher(ros::NodeHandle _nh)
    {
      this->nh = _nh;
      this->jointStatePublisher =
	this->nh.advertise<sensor_msgs::JointState>("aero_joint_states", 1);
      this->strokeSubscriber =
	this->nh.subscribe("/aero_controller/state",
			   1, &AeroJointStatePublisher::StrokeCallback, this);
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
    void AeroJointStatePublisher::StrokeCallback
    (const pr2_controllers_msgs::JointTrajectoryControllerState _msg)
    {
      common::Stroke2Angle(this->jointState, _msg);
    }

    //////////////////////////////////////////////////
    void AeroJointStatePublisher::Main()
    {
      this->jointState.header.stamp = ros::Time::now();
      this->jointStatePublisher.publish(this->jointState);
    }
  }
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "aero_state_publisher_node");
  ros::NodeHandle nh;
  aero::world::AeroJointStatePublisherPtr aero;
  aero.reset(new aero::world::AeroJointStatePublisher(nh));
  ros::Rate loopRate(30);
  while (ros::ok())
    {
      aero->Main();
      ros::spinOnce();
      loopRate.sleep();
    }
  return 0;
}
