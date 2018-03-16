#include <aero_std/AeroMoveitInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "fullbody_test_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);
  robot->sendLifter(0.0,0.0);

  // set real robot's joint angles to the robot model in interface 
  robot->setRobotStateToCurrentState();

  aero::joint_angle_map joint_angles;

  // test elbow and shoulder joints
  double l_elbow_to = -1.745;
  double r_elbow_to = -1.745;
  double l_shoulder_p_to = -0.4;
  double r_shoulder_p_to = -0.4;
  double l_shoulder_r_to = 0.5;
  double r_shoulder_r_to = -0.5;
  double l_shoulder_y_to = 0.7;
  double r_shoulder_y_to = -0.7;

  robot->getRobotStateVariables(joint_angles);

  ROS_INFO("elbow joints");
  joint_angles[aero::joint::l_elbow] = l_elbow_to;
  joint_angles[aero::joint::r_elbow] = r_elbow_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(1000);

  ROS_INFO("shoulder pitch joints");
  joint_angles[aero::joint::l_shoulder_p] = l_shoulder_p_to;
  joint_angles[aero::joint::r_shoulder_p] = r_shoulder_p_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(1000);

  ROS_INFO("shoulder roll joints");
  joint_angles[aero::joint::l_shoulder_r] = l_shoulder_r_to;
  joint_angles[aero::joint::r_shoulder_r] = r_shoulder_r_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(1000);

  ROS_INFO("shoulder yaw joints");
  joint_angles[aero::joint::l_shoulder_y] = l_shoulder_y_to;
  joint_angles[aero::joint::r_shoulder_y] = r_shoulder_y_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(1000);

  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose(1000);


  // test wrist and neck joints and hand
  double l_wrist_r_to = -0.5;
  double r_wrist_r_to = 0.5;
  double l_wrist_y_to = -0.5;
  double r_wrist_y_to = 0.5;

  robot->getRobotStateVariables(joint_angles);

  ROS_INFO("wrist roll joints");
  joint_angles[aero::joint::l_wrist_r] = l_wrist_r_to;
  joint_angles[aero::joint::r_wrist_r] = r_wrist_r_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(1000);

  ROS_INFO("wrist yaw joints");
  joint_angles[aero::joint::l_wrist_y] = l_wrist_y_to;
  joint_angles[aero::joint::r_wrist_y] = r_wrist_y_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(1000);

  ROS_INFO("looking at left hand");
  robot->setLookAt(robot->getEEFPosition(aero::arm::larm, aero::eef::pick));
  robot->sendAngleVector(1000);

  ROS_INFO("testing left hand");
  robot->sendGrasp(aero::arm::larm);
  robot->openHand(aero::arm::larm);
  robot->sendHand(aero::arm::larm, 0.0);

  ROS_INFO("looking at right hand");
  robot->setLookAt(robot->getEEFPosition(aero::arm::rarm, aero::eef::pick));
  robot->sendAngleVector(1000);

  ROS_INFO("testing right hand");
  robot->sendGrasp(aero::arm::rarm);
  robot->openHand(aero::arm::rarm);
  robot->sendHand(aero::arm::rarm, 0.0);

  robot->resetLookAt();
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose(1000);


  // test waist
  double waist_y_l_to = 0.7;
  double waist_y_r_to = -0.7;
  double waist_p_to = 0.3;

  robot->getRobotStateVariables(joint_angles);

  ROS_INFO("waist yaw joint");
  joint_angles[aero::joint::waist_y] = waist_y_l_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(2000);
  joint_angles[aero::joint::waist_y] = waist_y_r_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(3000);
  joint_angles[aero::joint::waist_y] = 0.0;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(2000);

  ROS_INFO("waist pitch joint");
  joint_angles[aero::joint::waist_p] = waist_p_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(2000);

  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose(2000);


  // test lifter
  ROS_INFO("lifter x:0.0 z:-0.4");
  robot->sendLifter(0.0, -0.4, 3000);
  usleep(500 * 1000);

  ROS_INFO("lifter x:0.0 z:-0.2");
  robot->sendLifter(0.0, -0.2, 3000);
  usleep(500 * 1000);

  ROS_INFO("lifter x:-0.1 z:-0.2");
  robot->sendLifter(-0.1, -0.2, 3000);
  usleep(500 * 1000);

  ROS_INFO("lifter x:0.1 z:-0.2");
  robot->sendLifter(0.1, -0.2, 3000);
  usleep(500 * 1000);

  ROS_INFO("lifter x:0.0 z:0.0");
  robot->sendLifter(0.0, 0.0, 3000);
  usleep(500 * 1000);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
