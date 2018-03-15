#include <aero_std/AeroMoveitInterface.hh>

/// @file ik.cc
/// @brief how to solve inverse kinematics, and reach the hand to there.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "ik_sample_node");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterfacePtr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);

#if 0
  geometry_msgs::Pose pose1;// target pose
  pose1.position.x = 0.57;
  pose1.position.y = -0.15;
  pose1.position.z = 1.1;
  pose1.orientation.w = 1.0;
  pose1.orientation.x = 0.0;
  pose1.orientation.y = 0.0;
  pose1.orientation.z = 0.0;
  ROS_INFO("ik target");
  ROS_INFO("pos");
  ROS_INFO("x:%f y%f z:%f", pose1.position.x, pose1.position.y, pose1.position.z);
  ROS_INFO("rot");
  ROS_INFO("w:%f x:%f y%f z:%f", pose1.orientation.w, pose1.orientation.x, pose1.orientation.y, pose1.orientation.z);
#endif
  //aero::Vector3 pos(0.57, -0.15, 1.1);
  aero::Translation pos(0.57, -0.15, 1.1);
  aero::Quaternion  rot(1.0, 0.0, 0.0, 0.0);
  aero::Transform   pose1 = pos * rot;

  ROS_INFO("ik target");
  ROS_INFO("pos");
  ROS_INFO("x:%f y:%f z:%f", pose1.translation().x(), pose1.translation().y(), pose1.translation().z());
  ROS_INFO("rot");
  {
    aero::Quaternion qq(pose1.linear());
    ROS_INFO("w:%f x:%f y:%f z:%f", qq.w(), qq.x(), qq.y(), qq.z());
  }

  // setFromIK(arm, ikrange, pose, eef)
  // solve IK and set that answer to the robot model which interface has
  //
  // ikrange is to select move torso or not , move lifter or not
  // aero::ikrange::[arm, torso, lifter] is prepared
  //
  // eef is moving target in hand
  // aero::eef::grasp is the center of cylinder which hand is grasping
  // aero::eef::pick is the tip of finger when hand angle is zero
  bool ik_result = robot->setFromIK(aero::arm::rarm, aero::ikrange::torso, pose1, aero::eef::grasp);

  // you can use Eigen::Vector3d and Eigen::Quaternion instead of geometry_msgs::Pose for ik target.
  // sample code
  //
  // Eigen::Vector3d pos(0.57, -0.15, 1.1);
  // Eigen::Quaterniond qua(1.0, 0.0, 0.0, 0.0);
  // bool ik_result = robot->setFromIK(aero::arm::rarm, aero::ikrange::torso, pos, qua, aero::eef::grasp);
  //

  if (ik_result) {// if ik successed, send the joint values to real robot
    ROS_INFO("ik success !");
    robot->sendAngleVector(aero::arm::rarm, aero::ikrange::torso, 3000);
    sleep(3);
    ROS_INFO("reseting robot pose");
    robot->sendResetManipPose();
  } else {
    ROS_WARN("ik failed");
  }

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
