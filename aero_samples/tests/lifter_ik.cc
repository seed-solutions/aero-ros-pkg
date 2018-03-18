#include <aero_std/AeroMoveitInterface.hh>

ros::ServiceClient lifter_srv;

bool checkLifter(double _x, double _z)
{
  aero_startup::AeroTorsoController srv;
  srv.request.x = 0.0;
  srv.request.z = 0.0;
  srv.request.coordinate = "local:500";
  lifter_srv.call(srv);

  // expect 2mm resolution
  if (fabs(_x - srv.response.x * 0.001) < 0.002 && fabs(_z - srv.response.z * 0.001) < 0.002)
    return true;
  else
    return false;
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "stoa_atos_test");
  ros::NodeHandle nh;
  lifter_srv = nh.serviceClient<aero_startup::AeroTorsoController>("/aero_torso_controller");

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr controller(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  controller->setPoseVariables(aero::pose::reset_manip);
  controller->sendAngleVector(3000);
  controller->sendLifter(0.0, 0.0);
  sleep(3);

  // solve lifter I.K. in range
  double resolution = 0.01;
  for (int j = -25; j < -7; ++j)
    for (int i = -16; i < 17; ++i) {
      double x = i * resolution;
      double z = j * resolution;
      if (!controller->sendLifter(x, z, 1000)) {
        ROS_WARN("cannot solve %lf, %lf", x, z);
      } else {
        if (!checkLifter)
          ROS_ERROR("returns can solve but can't %lf, %lf", x, z);
        else
          ROS_INFO("solved %lf, %lf", x, z);
      }
    }

  ROS_INFO("reseting robot pose");
  controller->setPoseVariables(aero::pose::reset_manip);
  controller->sendAngleVector(3000);
  controller->sendLifter(0.0, 0.0);
  sleep(3);
  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
