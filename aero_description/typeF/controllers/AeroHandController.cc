#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <aero_startup/AeroHandController.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>
/*
  @define srv
  string hand
  string command
  float32 thre_fail
  float32 thre_warn
  float32 larm_angle
  float32 rarm_angle
  ---
  string status
*/
#define Right 12
#define Left 27
#define grasp 2
#define ungrasp 3
#define cancel 4

ros::Publisher pub;
ros::Publisher pub_script;

float theta[2]; // left, right
int executing_flg_left =1;
int executing_flg_right =1;

bool HandControl(aero_startup::AeroHandController::Request &req,
		 aero_startup::AeroHandController::Response &res)
{
  trajectory_msgs::JointTrajectory msg;
//  std_msgs::Int32 msg_script;
  std_msgs::Int16MultiArray msg_script;

  msg.points.resize(1);


  if (req.command == "grasp")
  {
    if (req.hand == "both")
    {

    }
    else if (req.hand == "left")
    {
	msg_script.data = {Left,grasp};
        executing_flg_left = 1; //executing_grasp_script
    }
    else if (req.hand == "right")
    {
	msg_script.data = {Right,grasp};
        executing_flg_right = 1; //executing_grasp_script
    }

    pub_script.publish(msg_script);
    usleep(static_cast<int32_t>(2.0 * 1000 * 1000));
    ros::spinOnce();
    std::string status_msg = "grasp success";


    if (req.hand == "left")
    {
      if (theta[0] < req.thre_warn) status_msg = "grasp bad";
      if (theta[0] > req.thre_fail) status_msg = "grasp failed";
    }
    else if (req.hand == "right")
    {
 
      if (theta[1] > -req.thre_warn) status_msg = "grasp bad";
      if (theta[1] < -req.thre_fail) status_msg = "grasp failed";
    }

    res.status = status_msg;
  }
  else if (req.command == "ungrasp")
  {
    if (req.hand == "both")
    {
 
    }
    else if (req.hand == "left")
    {
	msg_script.data = {Left,ungrasp};
        executing_flg_left = 0;
    }
    else if (req.hand == "right")
    {
	msg_script.data = {Right,ungrasp};
        executing_flg_right = 0; 
    }
    pub_script.publish(msg_script);
    usleep(static_cast<int32_t>(2.0 * 1000 * 1000));
    res.status = "ungrasp success";
  }
  else if (req.command == "grasp-angle")
  {

        if (req.hand == "both")
        {
          msg.joint_names = {"l_thumb_joint", "r_thumb_joint"};
          msg.points[0].positions.resize(2);
          msg.points[0].positions[0] = req.larm_angle * M_PI / 180;
          msg.points[0].positions[1] = req.rarm_angle * M_PI / 180;
        }
        else if (req.hand == "left")
        {
	  if (executing_flg_left == 1){
	    msg_script.data = {Left,cancel};
	    pub_script.publish(msg_script);
            usleep(static_cast<int32_t>(0.2 * 1000 * 1000));
            executing_flg_left= 0;
	  } 
          msg.joint_names = {"l_thumb_joint"};
          msg.points[0].positions.resize(1);
          msg.points[0].positions[0] = req.larm_angle * M_PI / 180;
        }
        else if (req.hand == "right")
        {
	  if (executing_flg_right == 1){
	    msg_script.data = {Right,cancel};
	    pub_script.publish(msg_script);
            usleep(static_cast<int32_t>(0.2 * 1000 * 1000));
            executing_flg_right = 0;
	  }
          msg.joint_names = {"r_thumb_joint"};
          msg.points[0].positions.resize(1);
          msg.points[0].positions[0] = req.rarm_angle * M_PI / 180;
        }
        msg.points[0].time_from_start = ros::Duration(1);
        pub.publish(msg);
        res.status = "grasp-angle success";
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
  pub_script = nh.advertise<std_msgs::Int16MultiArray>(
      "/aero_controller/hand_control", 10);

  ros::Subscriber sub = nh.subscribe(
      "/aero_controller/state", 10, &HandStateCallback);

  ros::spin();

  return 0;
}
