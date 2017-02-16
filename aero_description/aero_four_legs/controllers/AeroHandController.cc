#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <aero_startup/AeroHandController.h>
#include <aero_startup/AeroSendJoints.h>

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

ros::ServiceClient client;

bool HandControl(aero_startup::AeroHandController::Request &req,
		 aero_startup::AeroHandController::Response &res)
{
  aero_startup::AeroSendJoints srv;

  if (req.command == "grasp") {
    if (req.hand == "both") {
      srv.request.joint_names = {"l_thumb_joint", "r_thumb_joint"};
      srv.request.points.positions.resize(2);
      srv.request.points.positions[0] = 50.0 * M_PI / 180;
      srv.request.points.positions[1] = -50.0 * M_PI / 180;
    } else if (req.hand == "left") {
      srv.request.joint_names = {"l_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = 50.0 * M_PI / 180;
    } else if (req.hand == "right") {
      srv.request.joint_names = {"r_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = -50.0 * M_PI / 180;
    }
    srv.request.points.time_from_start = ros::Duration(0.5);
    client.call(srv);
    theta[0] = srv.response.points.positions[13];
    theta[1] = srv.response.points.positions[27];
    std::string status_msg = "grasp success";
    if (req.hand == "left") {
      if (theta[0] < req.thre_warn) status_msg = "grasp bad";
      if (theta[0] > req.thre_fail) status_msg = "grasp failed";
    } else if (req.hand == "right") {
      if (theta[1] > -req.thre_warn) status_msg = "grasp bad";
      if (theta[1] < -req.thre_fail) status_msg = "grasp failed";
    }
    res.status = status_msg;
  }

  else if (req.command == "ungrasp") {
    if (req.hand == "both") {
      srv.request.joint_names = {"l_thumb_joint", "r_thumb_joint"};
      srv.request.points.positions.resize(2);
      srv.request.points.positions[0] = -50.0 * M_PI / 180;
      srv.request.points.positions[1] = 50.0 * M_PI / 180;
    } else if (req.hand == "left") {
      srv.request.joint_names = {"l_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = -50.0 * M_PI / 180;
    } else if (req.hand == "right") {
      srv.request.joint_names = {"r_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = 50.0 * M_PI / 180;
    }
    srv.request.points.time_from_start = ros::Duration(0.5);
    client.call(srv);
    res.status = "ungrasp success";
  }

  else if (req.command == "grasp-angle") {
    if (req.hand == "both") {
      srv.request.joint_names = {"l_thumb_joint", "r_thumb_joint"};
      srv.request.points.positions.resize(2);
      srv.request.points.positions[0] = req.larm_angle * M_PI / 180;
      srv.request.points.positions[1] = req.rarm_angle * M_PI / 180;
    } else if (req.hand == "left") {
      srv.request.joint_names = {"l_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = req.larm_angle * M_PI / 180;
    } else if (req.hand == "right") {
      srv.request.joint_names = {"r_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = req.rarm_angle * M_PI / 180;
    }
    srv.request.points.time_from_start = ros::Duration(0.5);
    client.call(srv);
    res.status = "grasp-angle success";
  }
 
  return true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_hand_controller");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService(
      "/aero_hand_controller", HandControl);
  client = nh.serviceClient<aero_startup::AeroSendJoints>(
      "/aero_controller/send_joints");

  ros::spin();

  return 0;
}
