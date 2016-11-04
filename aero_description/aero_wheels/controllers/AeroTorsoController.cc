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
static const double stroke_per_sec = 20.0; // mm/sec

double x_now;
double z_now;
double stroke_x_now;
double stroke_z_now;

ros::Publisher pub;

float LegTable(float _angle);

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

bool TorsoKinematics(aero_startup::AeroTorsoController::Request &req,
		     aero_startup::AeroTorsoController::Response &res)
{
  float goal_position_x = 0.0;
  float goal_position_z = 0.0;

  if (req.coordinate == "world") {
    goal_position_x = x_origin + req.x;
    goal_position_z = z_origin + req.z;
  } else if (req.coordinate == "local") {
    goal_position_x = x_now + req.x;
    goal_position_z = z_now + req.z;
  } else {
    res.status = "unexpected coordinate";
    return true;
  }

  double theta; // knee angle
  double phi; // ankle angle, should be > 0

  if (!SolveIK(goal_position_x, goal_position_z, theta, phi, res))
    return true; // failed ik

  trajectory_msgs::JointTrajectory msg;
  msg.joint_names = {"hip_joint", "knee_joint"};
  msg.points.resize(1);

  if (z_now <= (z_origin - 200) && goal_position_z >= z_now) {
    double theta0, phi0;
    if (!SolveIK(goal_position_x, z_now + 100, theta0, phi0, res)) {
      double try_x = static_cast<int>(goal_position_x / 10) * 10;
      double d_x = goal_position_x > 0 ? -10.0 : 10.0;
      while (!SolveIK(try_x, z_now + 100, theta0, phi0, res))
        try_x += d_x; // eventually should be able to solve
    }
    msg.points[0].positions = {theta0 - phi0, theta0};
    msg.points[0].time_from_start = ros::Duration(5.0); // go slowly
    pub.publish(msg);
    usleep(500 * 1000); // 500 ms
    msg.points[0].positions = {theta - phi, theta};
    msg.points[0].time_from_start = ros::Duration(1.0); // max speed
    pub.publish(msg);
  } else {
    msg.points[0].positions = {theta - phi, theta};
    msg.points[0].time_from_start = ros::Duration(1.0); // max speed
    pub.publish(msg);
  }

  float goal_stroke_x = LegTable(180.0 / M_PI * phi);
  float goal_stroke_z = LegTable(180.0 / M_PI * (theta - phi));
  res.time_sec = std::sqrt(std::pow(stroke_x_now - goal_stroke_x, 2) +
      std::pow(stroke_z_now - goal_stroke_z, 2)) / stroke_per_sec;

  x_now = goal_position_x;
  z_now = goal_position_z;
  stroke_x_now = goal_stroke_x;
  stroke_z_now = goal_stroke_z;

  res.status = "success";
  res.x = x_now - x_origin;
  res.z = z_now - z_origin;
  return true;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_torso_controller");
  ros::NodeHandle nh;

  x_now = 0.0;
  z_now = z_origin;

  stroke_x_now = LegTable(0.0);
  stroke_z_now = LegTable(0.0);

  ros::ServiceServer service = nh.advertiseService(
      "/aero_torso_controller", TorsoKinematics);
  pub = nh.advertise<trajectory_msgs::JointTrajectory>(
      "/aero_controller/command", 100);

  ros::spin();

  return 0;
}

float LegTable (float _angle)
{
  int roundedAngle = static_cast<int>(_angle);
  if (_angle > roundedAngle + 0.001) ++roundedAngle;
  float stroke;
  float interval;

  switch (roundedAngle) {
    case 0:
      stroke = 0;
      interval = 0.0000;
      break;
    case 1:
      stroke = .6230;
      interval = 0.6230;
      break;
    case 2:
      stroke = 1.2600;
      interval = 0.6370;
      break;
    case 3:
      stroke = 1.9090;
      interval = 0.6490;
      break;
    case 4:
      stroke = 2.5700;
      interval = 0.6610;
      break;
    case 5:
      stroke = 3.2420;
      interval = 0.6720;
      break;
    case 6:
      stroke = 3.9250;
      interval = 0.6830;
      break;
    case 7:
      stroke = 4.6180;
      interval = 0.6930;
      break;
    case 8:
      stroke = 5.3190;
      interval = 0.7010;
      break;
    case 9:
      stroke = 6.0290;
      interval = 0.7100;
      break;
    case 10:
      stroke = 6.7480;
      interval = 0.7190;
      break;
    case 11:
      stroke = 7.4730;
      interval = 0.7250;
      break;
    case 12:
      stroke = 8.2050;
      interval = 0.7320;
      break;
    case 13:
      stroke = 8.9440;
      interval = 0.7390;
      break;
    case 14:
      stroke = 9.6890;
      interval = 0.7450;
      break;
    case 15:
      stroke = 10.4380;
      interval = 0.7490;
      break;
    case 16:
      stroke = 11.1930;
      interval = 0.7550;
      break;
    case 17:
      stroke = 11.9530;
      interval = 0.7600;
      break;
    case 18:
      stroke = 12.7160;
      interval = 0.7630;
      break;
    case 19:
      stroke = 13.4830;
      interval = 0.7670;
      break;
    case 20:
      stroke = 14.2530;
      interval = 0.7700;
      break;
    case 21:
      stroke = 15.0260;
      interval = 0.7730;
      break;
    case 22:
      stroke = 15.8020;
      interval = 0.7760;
      break;
    case 23:
      stroke = 16.5800;
      interval = 0.7780;
      break;
    case 24:
      stroke = 17.3600;
      interval = 0.7800;
      break;
    case 25:
      stroke = 18.1420;
      interval = 0.7820;
      break;
    case 26:
      stroke = 18.9240;
      interval = 0.7820;
      break;
    case 27:
      stroke = 19.7080;
      interval = 0.7840;
      break;
    case 28:
      stroke = 20.4930;
      interval = 0.7850;
      break;
    case 29:
      stroke = 21.2780;
      interval = 0.7850;
      break;
    case 30:
      stroke = 22.0630;
      interval = 0.7850;
      break;
    case 31:
      stroke = 22.8490;
      interval = 0.7860;
      break;
    case 32:
      stroke = 23.6340;
      interval = 0.7850;
      break;
    case 33:
      stroke = 24.4190;
      interval = 0.7850;
      break;
    case 34:
      stroke = 25.2030;
      interval = 0.7840;
      break;
    case 35:
      stroke = 25.9860;
      interval = 0.7830;
      break;
    case 36:
      stroke = 26.7680;
      interval = 0.7820;
      break;
    case 37:
      stroke = 27.5480;
      interval = 0.7800;
      break;
    case 38:
      stroke = 28.3270;
      interval = 0.7790;
      break;
    case 39:
      stroke = 29.1050;
      interval = 0.7780;
      break;
    case 40:
      stroke = 29.8810;
      interval = 0.7760;
      break;
    case 41:
      stroke = 30.6540;
      interval = 0.7730;
      break;
    case 42:
      stroke = 31.4260;
      interval = 0.7720;
      break;
    case 43:
      stroke = 32.1950;
      interval = 0.7690;
      break;
    case 44:
      stroke = 32.9610;
      interval = 0.7660;
      break;
    case 45:
      stroke = 33.7250;
      interval = 0.7640;
      break;
    case 46:
      stroke = 34.4870;
      interval = 0.7620;
      break;
    case 47:
      stroke = 35.2450;
      interval = 0.7580;
      break;
    case 48:
      stroke = 36.0000;
      interval = 0.7550;
      break;
    case 49:
      stroke = 36.7520;
      interval = 0.7520;
      break;
    case 50:
      stroke = 37.5000;
      interval = 0.7480;
      break;
    case 51:
      stroke = 38.2450;
      interval = 0.7450;
      break;
    case 52:
      stroke = 38.9860;
      interval = 0.7410;
      break;
    case 53:
      stroke = 39.7240;
      interval = 0.7380;
      break;
    case 54:
      stroke = 40.4580;
      interval = 0.7340;
      break;
    case 55:
      stroke = 41.1880;
      interval = 0.7300;
      break;
    case 56:
      stroke = 41.9130;
      interval = 0.7250;
      break;
    case 57:
      stroke = 42.6350;
      interval = 0.7220;
      break;
    case 58:
      stroke = 43.3520;
      interval = 0.7170;
      break;
    case 59:
      stroke = 44.0640;
      interval = 0.7120;
      break;
    case 60:
      stroke = 44.7730;
      interval = 0.7090;
      break;
    case 61:
      stroke = 45.4760;
      interval = 0.7030;
      break;
    case 62:
      stroke = 46.1750;
      interval = 0.6990;
      break;
    case 63:
      stroke = 46.8690;
      interval = 0.6940;
      break;
    case 64:
      stroke = 47.5580;
      interval = 0.6890;
      break;
    case 65:
      stroke = 48.2420;
      interval = 0.6840;
      break;
    case 66:
      stroke = 48.9210;
      interval = 0.6790;
      break;
    case 67:
      stroke = 49.5950;
      interval = 0.6740;
      break;
    case 68:
      stroke = 50.2630;
      interval = 0.6680;
      break;
    case 69:
      stroke = 50.9260;
      interval = 0.6630;
      break;
    case 70:
      stroke = 51.5830;
      interval = 0.6570;
      break;
    case 71:
      stroke = 52.2350;
      interval = 0.6520;
      break;
    case 72:
      stroke = 52.8820;
      interval = 0.6470;
      break;
    case 73:
      stroke = 53.5220;
      interval = 0.6400;
      break;
    case 74:
      stroke = 54.1570;
      interval = 0.6350;
      break;
    case 75:
      stroke = 54.7860;
      interval = 0.6290;
      break;
    case 76:
      stroke = 55.4090;
      interval = 0.6230;
      break;
    case 77:
      stroke = 56.0260;
      interval = 0.6170;
      break;
    case 78:
      stroke = 56.6370;
      interval = 0.6110;
      break;
    case 79:
      stroke = 57.2420;
      interval = 0.6050;
      break;
    case 80:
      stroke = 57.8410;
      interval = 0.5990;
      break;
    case 81:
      stroke = 58.4330;
      interval = 0.5920;
      break;
    case 82:
      stroke = 59.0190;
      interval = 0.5860;
      break;
    case 83:
      stroke = 59.5990;
      interval = 0.5800;
      break;
    case 84:
      stroke = 60.1720;
      interval = 0.5730;
      break;
    case 85:
      stroke = 60.7380;
      interval = 0.5660;
      break;
    case 86:
      stroke = 61.2980;
      interval = 0.5600;
      break;
    case 87:
      stroke = 61.8510;
      interval = 0.5530;
      break;
    case 88:
      stroke = 62.3970;
      interval = 0.5460;
      break;
    case 89:
      stroke = 62.9370;
      interval = 0.5400;
      break;
    case 90:
      stroke = 63.4700;
      interval = 0.5330;
      break;
    default: return 0.0;
    }

  return stroke - (roundedAngle - _angle) * interval;
}
