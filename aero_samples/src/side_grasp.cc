#include <aero_std/AeroMoveitInterface.hh>
#include <aero_std/SideGrasp-inl.hh>

/// @file side_grasp.cc
/// @brief how to use SideGrasp. This function wrapps sequence of grasping object from side of it.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "side_grasp_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  //robot->sendResetManipPose();
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);


  // grasp object from side
  Eigen::Vector3d obj;// target object's position
  obj << 0.5, 0.0, 1.1;

  aero::SideGrasp side;// grasping information
  side.arm = aero::arm::either;
  side.object_position = obj;
  side.height = 0.2;

  auto req = aero::Grasp<aero::SideGrasp>(side);
  req.mid_ik_range = aero::ikrange::torso;
  req.end_ik_range = aero::ikrange::torso;


  robot->openHand(req.arm);

  if (//robot->sendPickIK(req) // TODO: not implemented yet
      true) {
    ROS_INFO("success");
    robot->sendGrasp(req.arm);
    sleep(3);
    robot->openHand(req.arm);
    sleep(1);
    ROS_INFO("reseting robot pose");
    robot->setPoseVariables(aero::pose::reset_manip);
    robot->sendAngleVector(3000);
    sleep(3);
  }
  else ROS_INFO("failed");

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
