#ifndef AERO_STATE_PUBLISHER_H_
#define AERO_STATE_PUBLISHER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

namespace aero
{
  namespace controller_manager
  {
    class AeroJointStatePublisher
    {
    public: explicit AeroJointStatePublisher(ros::NodeHandle _nh);

    public: ~AeroJointStatePublisher();

    public: void Main();

    private: void Init();

    private: void JointStateCallback(
        const pr2_controllers_msgs::JointTrajectoryControllerState _msg);

    private: ros::NodeHandle nh_;

    private: ros::Publisher joint_state_publisher_;

    private: ros::Subscriber stroke_subscriber_;

    private: sensor_msgs::JointState joint_state_;
    };

    typedef std::shared_ptr<AeroJointStatePublisher>
        AeroJointStatePublisherPtr;
  }
}

#endif
