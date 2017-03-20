#include <aero_std/AeroMoveitInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "ik_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr interface(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  interface->sendResetManipPose();
  sleep(1);


  geometry_msgs::Pose pose1;// target pose
  pose1.position.x = 0.57;
  pose1.position.y = -0.15;
  pose1.position.z = 0.73;
  pose1.orientation.w = 1.0;
  pose1.orientation.x = 0.0;
  pose1.orientation.y = 0.0;
  pose1.orientation.z = 0.0;

  // setFromIK(arm, ikrange, pose, eef)
  // solve IK and set that answer to the robot model which interface has
  //
  // ikrange is to select move torso or not , move lifter or not
  // aero::ikrange::[arm, torso, lifter] is prepared
  //
  // eef is moving target in hand
  // aero::eef::grasp is the center of cylinder which hand is grasping
  // aero::eef::pick is the tip of finger when hand angle is zero
  bool ik_result = interface->setFromIK(aero::arm::rarm, aero::ikrange::torso, pose1, aero::eef::grasp);

  if (ik_result) {// if ik successed, send the joint values to real robot
    ROS_INFO("ik success !");
    interface->sendAngleVector(aero::arm::rarm, aero::ikrange::torso, 3000);
    sleep(3);
    ROS_INFO("reseting robot pose");
    interface->sendResetManipPose();
  } else {
    ROS_WARN("ik failed");
  }

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
