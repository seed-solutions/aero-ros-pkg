#include <ros/ros.h>
#include "pr2_controllers_msgs/JointTrajectoryControllerState.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>

geometry_msgs::Point position;
geometry_msgs::Quaternion orientation;

bool failed;
std::vector<float> robot_state;

void RobotStates(
    const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& _msg)
{
  bool robot_had_action = false;

  if (robot_state.size() == _msg->joint_names.size())
  { // robot_state is already initialized
    for (unsigned int i = 0; i < robot_state.size(); ++i)
      if (fabs(robot_state[i] - _msg->actual.positions[i]) > 0.01)
      { // robot_state differs from previous state
	robot_had_action = true;
	robot_state[i] = _msg->actual.positions[i];
      }
  }
  else
  { // robot_state is not initialized yet
    robot_state.clear();
    robot_state.resize(_msg->joint_names.size());
    for (unsigned int i = 0; i < robot_state.size(); ++i)
      robot_state[i] = _msg->actual.positions[i];
  }

  // only update tf when robot had action or getting tf failed
  if (!robot_had_action && !failed) return;

  static tf::TransformListener tl;
  ros::Time now = ros::Time::now();
  tl.waitForTransform("leg_base_link", "ps4eye_frame",
		      now, ros::Duration(2.0));
  try
  {
    tf::StampedTransform st;
    tl.lookupTransform("leg_base_link", "ps4eye_frame", now, st);
    position.x = st.getOrigin().x();
    position.y = st.getOrigin().y();
    position.z = st.getOrigin().z();
    orientation.x = st.getRotation().w();
    orientation.y = st.getRotation().x();
    orientation.z = st.getRotation().y();
    orientation.w = st.getRotation().z();
    failed = false;
  }
  catch (std::exception e)
  {
    ROS_ERROR("failed tf listen");
    failed = true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_coords_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub =
      nh.advertise<geometry_msgs::Pose>("/matrix/base_to_eye", 1000);
  ros::Subscriber sub =
      nh.subscribe("/aero_controller/state", 1000, &RobotStates);

  position.x = 0;
  position.y = 0;
  position.z = 0;
  orientation.x = 1;
  orientation.y = 0;
  orientation.z = 0;
  orientation.w = 0;
  failed = true;

  ros::Rate r(30);

  while(ros::ok())
  {
    ros::spinOnce();
    geometry_msgs::Pose msg;
    msg.position = position;
    msg.orientation = orientation;
    pub.publish(msg);
    r.sleep();
  }
}
