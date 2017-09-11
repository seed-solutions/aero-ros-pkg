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

aero_startup::AeroSendJoints::Response GraspAngle
(std::string hand, float larm_angle, float rarm_angle, float time=0.5)
{
  aero_startup::AeroSendJoints srv;
  if (hand == "both") {
    srv.request.joint_names = {"l_thumb_joint", "r_thumb_joint"};
    srv.request.points.positions.resize(2);
    srv.request.points.positions[0] = larm_angle * M_PI / 180;
    srv.request.points.positions[1] = rarm_angle * M_PI / 180;
  } else if (hand == "left") {
    if (executing_flg_left == 1) {
      aero_startup::AeroGraspController g_srv;
      g_srv.request.script = {Left, cancel};
      g_srv.request.power = (100 << 8) + 30;
      g_client.call(g_srv);
      executing_flg_left= 0;
    }
    srv.request.joint_names = {"l_thumb_joint"};
    srv.request.points.positions.resize(1);
    srv.request.points.positions[0] = larm_angle * M_PI / 180;
  } else if (hand == "right") {
    if (executing_flg_right == 1) {
      aero_startup::AeroGraspController g_srv;
      g_srv.request.script = {Right, cancel};
      g_srv.request.power = (100 << 8) + 30;
      g_client.call(g_srv);
      executing_flg_right = 0;
    }
    srv.request.joint_names = {"r_thumb_joint"};
    srv.request.points.positions.resize(1);
    srv.request.points.positions[0] = rarm_angle * M_PI / 180;
  }
  srv.request.points.time_from_start = ros::Duration(time);
  client.call(srv);

  return srv.response;
};

void OpenHand(std::string hand)
{
  aero_startup::AeroGraspController g_srv;

  if (hand == "left") {
    g_srv.request.script = {Left, ungrasp};
    executing_flg_left = 0;
  } else if (hand == "right") {
    g_srv.request.script = {Right, ungrasp};
    executing_flg_right = 0;
  }
  g_srv.request.power = (100 << 8) + 30;
  g_client.call(g_srv);
}

bool HandControl(aero_startup::AeroHandController::Request &req,
		 aero_startup::AeroHandController::Response &res)
{
  aero_startup::AeroGraspController g_srv;

  std::string cmd = req.command;
  std::string power = "";

  auto pos = req.command.find(":");
  if (pos != std::string::npos) {
    cmd = std::string(req.command.begin(), req.command.begin() + pos);
    power = std::string(req.command.begin() + pos + 1, req.command.end());
  }

  if (cmd == "grasp") {
    // because grasp is really really slow, first grasp-angle
    GraspAngle(req.hand, 0.0, 0.0);

    if (req.hand == "left") {
      g_srv.request.script = {Left, grasp};
      executing_flg_left = 1; //executing_grasp_script
    } else if (req.hand == "right") {
      g_srv.request.script = {Right, grasp};
      executing_flg_right = 1; //executing_grasp_script
    }

    if (power != "")
      g_srv.request.power = (std::stoi(power) << 8) + 30;
    else
      g_srv.request.power = (100 << 8) + 30;

    g_client.call(g_srv);
    std::string status_msg = "grasp success";
    if (req.hand == "left") {
      if (g_srv.response.angles[0] < req.thre_warn) status_msg = "grasp bad";
      if (g_srv.response.angles[0] > req.thre_fail) status_msg = "grasp failed";
    } else if (req.hand == "right") {
      if (g_srv.response.angles[1] > -req.thre_warn) status_msg = "grasp bad";
      if (g_srv.response.angles[1] < -req.thre_fail) status_msg = "grasp failed";
    }

    res.status = status_msg;
  }

  else if (cmd == "ungrasp") {
    OpenHand(req.hand);
    res.status = "ungrasp success";
  }

  else if (cmd == "grasp-angle") {
    GraspAngle(req.hand, req.larm_angle, req.rarm_angle);
    res.status = "grasp-angle success";
  }

  else if (cmd == "grasp-fast") {
    // grasp till angle, check if grasp is okay, then hold
    auto joints = GraspAngle(req.hand, 0.0, 0.0, 1.2); // grasp takes about 1 sec

    if (req.hand == "left") {
      int at = static_cast<int> // should always be found
        (std::find(joints.joint_names.begin(), joints.joint_names.end(),
                   "l_thumb_joint") - joints.joint_names.begin());
      ROS_INFO("%f", joints.points.positions.at(at));
      if (fabs(joints.points.positions.at(at)) < 0.05) {
        OpenHand(req.hand);
        res.status = "grasp failed";
        return true;
      }
      g_srv.request.script = {Left, grasp};
      executing_flg_left = 1; //executing_grasp_script
    } else if (req.hand == "right") {
      int at = static_cast<int> // should always be found
        (std::find(joints.joint_names.begin(), joints.joint_names.end(),
                   "r_thumb_joint") - joints.joint_names.begin());
      if (fabs(joints.points.positions.at(at)) < 0.05) {
        OpenHand(req.hand);
        res.status = "grasp failed";
        return true;
      }
      g_srv.request.script = {Right, grasp};
      executing_flg_right = 1; //executing_grasp_script
    }

    if (power != "")
      g_srv.request.power = (std::stoi(power) << 8) + 30;
    else
      g_srv.request.power = (100 << 8) + 30;

    g_client.call(g_srv);
    std::string status_msg = "grasp success";
    if (req.hand == "left")
      if (g_srv.response.angles[0] > req.thre_fail) status_msg = "grasp failed";
    else if (req.hand == "right")
      if (g_srv.response.angles[1] < -req.thre_fail) status_msg = "grasp failed";

    res.status = status_msg;
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
  g_client = nh.serviceClient<aero_startup::AeroGraspController>(
      "/aero_controller/grasp_control");

  ros::spin();

  return 0;
}
