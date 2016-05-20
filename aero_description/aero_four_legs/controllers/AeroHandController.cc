#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <aero_startup/AeroHandController.h>

/*
  @define srv
  string hand
  string command
  float32 thre_fail
  float32 thre_warn
  ---
  string status
*/

ros::Publisher pub;
float theta[2]; // left, right

bool HandControl(aero_startup::AeroHandController::Request &req,
		 aero_startup::AeroHandController::Response &res)
{
  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);

  if (req.command == "grasp")
  {
    if (req.hand == "both")
    {
      msg.joint_names = {"l_thumb_joint", "r_thumb_joint"};
      msg.points[0].positions.resize(2);
      msg.points[0].positions[0] = 50.0 * M_PI / 180;
      msg.points[0].positions[1] = -50.0 * M_PI / 180;
    }
    else if (req.hand == "left")
    {
      msg.joint_names = {"l_thumb_joint"};
      msg.points[0].positions.resize(1);
      msg.points[0].positions[0] = 50.0 * M_PI / 180;
    }
    else if (req.hand == "right")
    {
      msg.joint_names = {"r_thumb_joint"};
      msg.points[0].positions.resize(1);
      msg.points[0].positions[0] = -50.0 * M_PI / 180;
    }
    msg.points[0].time_from_start = ros::Duration(0.5);
    pub.publish(msg);
    usleep(static_cast<int32_t>(2.0 * 1000 * 1000));
    ros::spinOnce();
    std::string status_msg = "grasp success";
    if (req.hand == "both")
    {
      msg.points[0].positions[0] = std::min(theta[0]+0.1, 50.0*M_PI/180);
      msg.points[0].positions[1] = std::max(theta[1]-0.1, -50.0*M_PI/180);
    }
    else if (req.hand == "left")
    {
      msg.joint_names = {"l_thumb_joint"};
      msg.points[0].positions.resize(1);
      msg.points[0].positions[0] = std::min(theta[0]+0.1, 50.0*M_PI/180);
      if (theta[0] < req.thre_warn) status_msg = "grasp bad";
      if (theta[0] > req.thre_fail) status_msg = "grasp failed";
    }
    else if (req.hand == "right")
    {
      msg.joint_names = {"r_thumb_joint"};
      msg.points[0].positions.resize(1);
      msg.points[0].positions[0] = std::max(theta[1]-0.1, -50.0*M_PI/180);
      if (theta[1] > -req.thre_warn) status_msg = "grasp bad";
      if (theta[1] < -req.thre_fail) status_msg = "grasp failed";
    }
    msg.points[0].time_from_start = ros::Duration(0.1);
    pub.publish(msg);
    res.status = status_msg;
  }
  else if (req.command == "ungrasp")
  {
    if (req.hand == "both")
    {
      msg.joint_names = {"l_thumb_joint", "r_thumb_joint"};
      msg.points[0].positions.resize(2);
      msg.points[0].positions[0] = -50.0 * M_PI / 180;
      msg.points[0].positions[1] = 50.0 * M_PI / 180;
    }
    else if (req.hand == "left")
    {
      msg.joint_names = {"l_thumb_joint"};
      msg.points[0].positions.resize(1);
      msg.points[0].positions[0] = -50.0 * M_PI / 180;
    }
    else if (req.hand == "right")
    {
      msg.joint_names = {"r_thumb_joint"};
      msg.points[0].positions.resize(1);
      msg.points[0].positions[0] = 50.0 * M_PI / 180;
    }
    msg.points[0].time_from_start = ros::Duration(0.1);
    pub.publish(msg);
    res.status = "ungrasp success";
  }
 
  return true;
};

void HandStateCallback(
    const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& _msg)
{
  theta[0] = _msg->actual.positions[13];
  theta[1] = _msg->actual.positions[27];
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_hand_controller");
  ros::NodeHandle nh;

  theta[0] = 0.0;
  theta[1] = 0.0;

  ros::ServiceServer service = nh.advertiseService(
      "/aero_hand_controller", HandControl);
  pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/aero_controller/command", 100);
  ros::Subscriber sub = nh.subscribe(
      "/aero_controller/state", 10, &HandStateCallback);

  ros::spin();

  return 0;
}
