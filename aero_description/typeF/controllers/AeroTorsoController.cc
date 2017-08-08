#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <aero_startup/AeroTorsoController.h>
#include <aero_startup/AeroSendJoints.h>

static const double link1_length = 250.0;
static const double link2_length = 250.0;
static const double x_origin = 0.0;
static const double z_origin = 500.0;

ros::Publisher pub;
ros::ServiceClient get_joints;

bool SolveIK(float goal_position_x, float goal_position_z,
             double &theta, double &phi,
             aero_startup::AeroTorsoController::Response &res)
{
  double to_end_position =
    goal_position_x * goal_position_x + goal_position_z * goal_position_z;

  theta =
    acos((to_end_position -
	  link1_length * link1_length - link2_length * link2_length) /
	 (2 * link1_length * link2_length));
  if (theta < 0) theta = -theta;  // expects theta > 0

  if (theta >= M_PI) {
    res.status =
      "invalid goal theta = " + boost::lexical_cast<std::string>(theta);
    return false;
  }

  double xi = acos(goal_position_x / sqrt(to_end_position)); // > 0
  double k1 = link1_length + link2_length * cos(theta);
  double k2 = link2_length * sin(theta);
  double gamma = atan(k2 / k1); // > 0
  if (k1 < 0) gamma += M_PI;

  phi = xi + gamma - M_PI * 0.5;

  if (phi < 0 || phi > theta || phi > M_PI * 0.5) {
    res.status =
      "invalid goal phi = " + boost::lexical_cast<std::string>(phi);
    return false;
  }

  if (theta != theta || phi != phi) {
    res.status = "nan value error";
    return false;
  }

  return true;
}

std::string ParseTimeSpecifiedMode(std::string &s, int &time)
{
  // check grammar
  auto pos = s.find(":");
  if (pos == std::string::npos || pos == (s.length() - 1))
    return "";
  // check if time is number
  auto it = s.begin() + pos + 1;
  while (it != s.end() && std::isdigit(*it)) ++it;
  if (it != s.end())
    return "";
  // check if time is valid
  time = std::stoi(std::string(s.begin() + pos + 1, s.end()));
  if (time <= 0)
    return "";
  // return coordinate
  return std::string(s.begin(), s.begin() + pos);
}

bool TorsoControl(aero_startup::AeroTorsoController::Request &req,
                  aero_startup::AeroTorsoController::Response &res)
{
  float goal_position_x = 0.0;
  float goal_position_z = 0.0;

  // once upon a time, there was an auto-speed calculation mode
  // however, because the hardware changes so often, the mode was removed
  int time_ms = 10000; // default time when not specified

  // update x_now and z_now
  aero_startup::AeroSendJoints srv;
  srv.request.reset_status = false;
  get_joints.call(srv);
  float hip_ref = srv.response.points.positions.at(29);
  float knee_ref = srv.response.points.positions.at(30);
  float x_now =
    link1_length * sin(hip_ref) - link2_length * sin(knee_ref - hip_ref);
  float z_now =
    link1_length * cos(hip_ref) + link2_length * cos(knee_ref - hip_ref);

  if (req.coordinate == "world") {
    goal_position_x = x_origin + req.x;
    goal_position_z = z_origin + req.z;
  } else if (req.coordinate == "local") {
    goal_position_x = x_now + req.x;
    goal_position_z = z_now + req.z;
  } else {
    // time specified mode; world:1000, local:500
    std::string coords = ParseTimeSpecifiedMode(req.coordinate, time_ms);
    if (coords == "world") {
      goal_position_x = x_origin + req.x;
      goal_position_z = z_origin + req.z;
    } else if (coords == "local") {
      goal_position_x = x_now + req.x;
      goal_position_z = z_now + req.z;
    } else {
      res.status = "unexpected coordinate";
      return true;
    }
  }

  double theta; // knee angle
  double phi; // ankle angle, should be > 0

  if (!SolveIK(goal_position_x, goal_position_z, theta, phi, res))
    return true; // failed ik

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = {"hip_joint", "knee_joint"};
  msg.points.resize(1);

  msg.points[0].positions = {theta - phi, theta};
  msg.points[0].time_from_start = ros::Duration(time_ms / 1000.0);
  pub.publish(msg);

  res.time_sec = time_ms / 1000.0;

  res.status = "success";
  res.x = goal_position_x - x_origin;
  res.z = goal_position_z - z_origin;
  return true;
};

bool TorsoKinematics(aero_startup::AeroTorsoController::Request &req,
		     aero_startup::AeroTorsoController::Response &res)
{
  float goal_position_x = x_origin + req.x;
  float goal_position_z = z_origin + req.z;

  double theta; // knee angle
  double phi; // ankle angle, should be > 0

  if (!SolveIK(goal_position_x, goal_position_z, theta, phi, res))
    return true; // failed ik

  res.status = "success";
  res.x = theta - phi; // hip_joint
  res.z = theta; // knee_joint
  return true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_torso_controller");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService(
      "/aero_torso_controller", TorsoControl);
  ros::ServiceServer kinematics_service = nh.advertiseService(
      "/aero_torso_kinematics", TorsoKinematics);

  get_joints = nh.serviceClient<aero_startup::AeroSendJoints>(
      "/aero_controller/get_joints");

  pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/aero_controller/command", 100);

  ros::spin();

  return 0;
}
