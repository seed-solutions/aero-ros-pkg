#ifndef _AERO_STATE_PUBLISHER_H_
#define _AERO_STATE_PUBLISHER_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

namespace aero
{
  namespace world {
    class AeroJointStatePublisher
    {
    public: explicit AeroJointStatePublisher(ros::NodeHandle _nh);

    public: ~AeroJointStatePublisher();

    public: void Main();

    private: void Init();

    private: void StrokeCallback(const pr2_controllers_msgs::JointTrajectoryControllerState _msg);

    private: ros::NodeHandle nh;

    private: ros::Publisher jointStatePublisher;

    private: ros::Subscriber strokeSubscriber;

    private: sensor_msgs::JointState jointState;
    };

    typedef boost::shared_ptr<AeroJointStatePublisher> AeroJointStatePublisherPtr;
  }
}

#endif
