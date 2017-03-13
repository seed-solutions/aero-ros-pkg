#include <ros/ros.h>
#include <aero_startup/AeroHandController.h>
#include <aero_startup/AeroGraspController.h>
#include <aero_startup/AeroSendJoints.h>

#define Right 12
#define Left 27
#define grasp 2
#define ungrasp 3
#define cancel 4

ros::ServiceClient client;
ros::ServiceClient g_client;

int executing_flg_left = 1;
int executing_flg_right = 1;

bool HandControl(aero_startup::AeroHandController::Request &req,
		 aero_startup::AeroHandController::Response &res)
{
  aero_startup::AeroSendJoints srv;
  aero_startup::AeroGraspController g_srv;

  std::string cmd = req.command;
  std::string power = "";

  auto pos = req.command.find(":");
  if (pos != std::string::npos) {
    cmd = std::string(req.command.begin(), req.command.begin() + pos);
    power = std::string(req.command.begin() + pos + 1, req.command.end());
  }

  if (cmd == "grasp") {
    if (req.hand == "left") {
      g_srv.request.script = {Left, grasp};
      executing_flg_left = 1; //executing_grasp_script
    } else if (req.hand == "right") {
      g_srv.request.script = {Right, grasp};
      executing_flg_right = 1; //executing_grasp_script
    }

    if (power != "")
      g_srv.request.power = (std::stoi(power) << 8) + 30;

    g_client.call(g_srv);
    std::string status_msg = "grasp success";
    if (req.hand == "left") {
      if (g_srv.response.angles[0] < req.thre_warn) status_msg = "grasp bad";
      if (g_srv.response.angles[1] > req.thre_fail) status_msg = "grasp failed";
    } else if (req.hand == "right") {
      if (g_srv.response.angles[1] > -req.thre_warn) status_msg = "grasp bad";
      if (g_srv.response.angles[1] < -req.thre_fail) status_msg = "grasp failed";
    }

    res.status = status_msg;
  }

  else if (cmd == "ungrasp") {
    if (req.hand == "left") {
      g_srv.request.script = {Left, ungrasp};
      executing_flg_left = 0;
    } else if (req.hand == "right") {
      g_srv.request.script = {Right, ungrasp};
      executing_flg_right = 0; 
    }
    g_client.call(g_srv);
    res.status = "ungrasp success";
  }

  else if (cmd == "grasp-angle") {
    if (req.hand == "both") {
      srv.request.joint_names = {"l_thumb_joint", "r_thumb_joint"};
      srv.request.points.positions.resize(2);
      srv.request.points.positions[0] = req.larm_angle * M_PI / 180;
      srv.request.points.positions[1] = req.rarm_angle * M_PI / 180;
    } else if (req.hand == "left") {
      if (executing_flg_left == 1) {
        g_srv.request.script = {Left, cancel};
        g_srv.request.power = (100 << 8) + 30;
        g_client.call(g_srv);
        executing_flg_left= 0;
      }
      srv.request.joint_names = {"l_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = req.larm_angle * M_PI / 180;
    } else if (req.hand == "right") {
      if (executing_flg_right == 1) {
        g_srv.request.script = {Right, cancel};
        g_srv.request.power = (100 << 8) + 30;
        g_client.call(g_srv);
        executing_flg_right = 0;
      }
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

// void HandStateCallback(
//     const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& _msg)
// {
//   theta[0] = _msg->actual.positions[13];
//   theta[1] = _msg->actual.positions[27];
// };

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_hand_controller");
  ros::NodeHandle nh;

  // theta[0] = 0.0;
  // theta[1] = 0.0;

  ros::ServiceServer service = nh.advertiseService(
      "/aero_hand_controller", HandControl);
  client = nh.serviceClient<aero_startup::AeroSendJoints>(
      "/aero_controller/send_joints");
  g_client = nh.serviceClient<aero_startup::AeroGraspController>(
      "/aero_controller/grasp_control");

  // ros::Subscriber sub = nh.subscribe(
  //     "/aero_controller/state", 10, &HandStateCallback);

  ros::spin();

  return 0;
}
