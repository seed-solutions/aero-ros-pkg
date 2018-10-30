#include <aero_std/AeroMoveitInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "stoa_atos_test");
  ros::NodeHandle nh;
  // lifter_srv = nh.serviceClient<aero_startup::AeroTorsoController>("/aero_torso_controller");

  // init robot interface
  aero::interface::AeroMoveitInterfacePtr controller(new aero::interface::AeroMoveitInterfaceDeprecated(nh));
  ROS_INFO("reseting robot pose");
  controller->setPoseVariables(aero::pose::reset_manip);
  controller->sendModelAngles(3000);
  controller->sendLifter(0.0, 0.0);
  sleep(3);

  // solve lifter I.K. in range
  double resolution = 0.01;
  for (int zz = -60; zz < 0; zz += 3) {
    for (int xx = -16; xx < 17; ++xx) {
      double x = xx * resolution;
      double z = zz * resolution;
      bool solved = false;
      controller->setJoint(aero::joint::ankle, 0.0);
      controller->setJoint(aero::joint::knee, 0.0);
      if (!controller->setLifter(x, z)) {
        ROS_WARN("cannot solve %lf, %lf", x, z);
        controller->setJoint(aero::joint::ankle, 0.0);
        controller->setJoint(aero::joint::knee, 0.0);
        if (!controller->setLifter(x, z)) {
          ROS_WARN("retry failed %lf, %lf", x, z);
        } else {
          solved = true;
        }
      } else {
        solved = true;
      }
      if (solved) {
        ROS_INFO("solved %lf, %lf => %lf %lf", x, z,
                 controller->getJoint(aero::joint::ankle),
                 controller->getJoint( aero::joint::knee));
      }
    }
  }

  ROS_INFO("reseting robot pose");
  controller->setPoseVariables(aero::pose::reset_manip);
  controller->sendModelAngles(3000);
  controller->sendLifter(0.0, 0.0);
  sleep(3);
  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
