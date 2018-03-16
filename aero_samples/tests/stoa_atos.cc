#include <aero_std/AeroMoveitInterface.hh>

void warnDiff(aero::interface::AeroMoveitInterfacePtr controller,
              aero::joint joint, double send_val)
{
  // get current angles
  aero::joint_angle_map get_angles;
  controller->setRobotStateToCurrentState();
  controller->getRobotStateVariables(get_angles);

  double diff = fabs(get_angles.at(joint) - send_val);
  if (diff < 0.02) // around 1 degree
    ROS_INFO("diff okay in %s %lf: %lf",
             aero::joint_map.at(joint).c_str(), send_val, diff);
  else
    ROS_WARN("bad diff in %s %lf: %lf",
             aero::joint_map.at(joint).c_str(), send_val, diff);
}

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "stoa_atos_test");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr controller(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  controller->sendResetManipPose();
  sleep(1);

  // check diff of stoa and atos for each joint
  float resolution = 0.01745; 
  for (auto j = aero::joint_map.begin(); j != aero::joint_map.end(); ++j) {
    if (j->first == aero::joint::lifter_x) // don't check lifter joints
      break;
    if (j->second.find("r_") == 0) // right is same as left so skip
      continue;
    if (j->second.find("_y_") != std::string::npos) // skip yaw
      continue;

    // find limit bounds
    auto bounds = controller->kinematic_model->getVariableBounds(j->second);

    aero::joint_angle_map joint_angles;
    controller->getResetManipPose(joint_angles);
    // take time for first send
    joint_angles.at(j->first) = bounds.min_position_;
    controller->setRobotStateVariables(joint_angles);
    controller->sendAngleVector(5000);
    usleep(500 * 1000);
    warnDiff(controller, j->first, joint_angles.at(j->first)); // get result
    joint_angles.at(j->first) += resolution;
    // for rest, send fast
    while (joint_angles.at(j->first) < bounds.max_position_) {
      controller->setRobotStateVariables(joint_angles);
      controller->sendAngleVector(100);
      usleep(500 * 1000);
      warnDiff(controller, j->first, joint_angles.at(j->first)); // get result
      joint_angles.at(j->first) += resolution;
    }
  }

  ROS_INFO("reseting robot pose");
  controller->sendResetManipPose();
  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
