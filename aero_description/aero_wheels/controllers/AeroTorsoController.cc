#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <aero_startup/AeroTorsoController.h>

/*
  @define srv
  float64 x
  float64 z
  string coordinate
  ---
  string status
  float64 x
  float64 z
  float64 time_sec
*/

static const double link1_length = 290.09;
static const double link2_length = 290.09;
static const double x_origin = 0.0;
static const double z_origin = 580.18; 

double x_now;
double z_now;

ros::Publisher pub;

bool TorsoKinematics(aero_startup::AeroTorsoController::Request &req,
		     aero_startup::AeroTorsoController::Response &res)
{
  float goal_position_x = 0.0;
  float goal_position_z = 0.0;

  if (req.coordinate == "world")
  {
    goal_position_x = x_origin + req.x;
    goal_position_z = z_origin + req.z;
  }
  else if (req.coordinate == "local")
  {
    goal_position_x = x_now + req.x;
    goal_position_z = z_now + req.z;
  }
  else
  {
    res.status = "unexpected coordinate";
    return true;
  }

  double to_end_position =
    goal_position_x * goal_position_x + goal_position_z * goal_position_z;

  double theta = // knee angle
    acos((to_end_position -
	  link1_length * link1_length - link2_length * link2_length) /
	 (2 * link1_length * link2_length));
  if (theta < 0) theta = -theta;  // expects theta > 0

  if (theta >= M_PI)
  {
    res.status =
      "invalid goal theta = " + boost::lexical_cast<std::string>(theta);
    return true;
  }

  double xi = acos(goal_position_x / sqrt(to_end_position)); // > 0
  double k1 = link1_length + link2_length * cos(theta);
  double k2 = link2_length * sin(theta);
  double gamma = atan(k2 / k1); // > 0
  if (k1 < 0) gamma += M_PI;
  double phi = 0.0; // ankle angle, should be > 0

  phi = xi + gamma - M_PI * 0.5;

  if (phi < 0 || phi > theta || phi > M_PI * 0.5)
  {
    res.status =
      "invalid goal phi = " + boost::lexical_cast<std::string>(phi);
    return true;
  }

  if (theta != theta || phi != phi)
  {
    res.status = "nan value error";
    return true;
  }

  float time_scale =
      (std::pow(x_now - goal_position_x, 2) +
       std::pow(z_now - goal_position_z, 2)) / 20000.0;
  time_scale = std::max(time_scale, static_cast<float>(1.0));

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = {"hip_joint", "knee_joint"};
  msg.points.resize(1);
  msg.points[0].positions = {theta-phi, theta};
  msg.points[0].time_from_start = ros::Duration(1.0 * time_scale);

  pub.publish(msg);

  x_now = goal_position_x;
  z_now = goal_position_z;

  res.status = "success";
  res.x = x_now - x_origin;
  res.z = z_now - z_origin;
  res.time_sec = time_scale;
  return true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_torso_controller");
  ros::NodeHandle nh;

  x_now = 0.0;
  z_now = z_origin;

  ros::ServiceServer service = nh.advertiseService(
      "/aero_torso_controller", TorsoKinematics);
  pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/aero_controller/command", 100);

  ros::spin();

  return 0;
}

