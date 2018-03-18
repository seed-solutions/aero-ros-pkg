#include <aero_std/AeroMoveitInterface.hh>

#define PJOINT(jname) {                                                 \
    ROS_DEBUG("a "#jname"_joint: %f", joint_angles[aero::joint::jname]); \
  }

#define ALLPJOINT() {                     \
    PJOINT(ankle);                        \
    PJOINT(knee);                         \
    PJOINT(l_elbow);                      \
    PJOINT(l_hand_y);                     \
    PJOINT(l_shoulder_p);                 \
    PJOINT(l_shoulder_r);                 \
    PJOINT(l_shoulder_y);                 \
    PJOINT(l_wrist_p);                    \
    PJOINT(l_wrist_r);                    \
    PJOINT(l_wrist_y);                    \
    PJOINT(r_elbow);                      \
    PJOINT(r_hand_y);                     \
    PJOINT(r_shoulder_p);                 \
    PJOINT(r_shoulder_r);                 \
    PJOINT(r_shoulder_y);                 \
    PJOINT(r_wrist_p);                    \
    PJOINT(r_wrist_r);                    \
    PJOINT(r_wrist_y);                    \
    PJOINT(waist_p);                      \
    PJOINT(waist_r);                      \
    PJOINT(waist_y);                      \
  }

#define ALLSJOINT() {                                           \
    robot_interface::joint_angle_map map;                       \
    robot->getRobotStateVariables(map);                         \
    for(auto it = map.begin(); it != map.end(); it++) {         \
      ROS_DEBUG("s %s: %f", it->first.c_str(), it->second);     \
    }                                                           \
  }

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "fullbody_test_sample_node");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  ROS_INFO("--- RESET MANIP POSE ---");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  // sleep(3);

  robot->sendLifter(0.0,0.0);

  // set real robot's joint angles to the robot model in interface 
  robot->setRobotStateToCurrentState();

  aero::joint_angle_map joint_angles;

  // test elbow and shoulder joints
  double l_elbow_to = -1.745;
  double r_elbow_to = -1.745;
  double l_shoulder_p_to = -0.4;
  double r_shoulder_p_to = -0.4;
  double l_shoulder_r_to =  0.5;
  double r_shoulder_r_to = -0.5;
  double l_shoulder_y_to =  0.7;
  double r_shoulder_y_to = -0.7;

  robot->getRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();

  ROS_INFO("--- ELBOW JOINTS ---");
  joint_angles[aero::joint::l_elbow] = l_elbow_to;
  joint_angles[aero::joint::r_elbow] = r_elbow_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendAngleVector(3000);


  ROS_INFO("--- SHOULDER PITCH JOINT ---");
  joint_angles[aero::joint::l_shoulder_p] = l_shoulder_p_to;
  joint_angles[aero::joint::r_shoulder_p] = r_shoulder_p_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendAngleVector(3000);


  ROS_INFO("--- SHOULDER ROLL JOINT ---");
  joint_angles[aero::joint::l_shoulder_r] = l_shoulder_r_to;
  joint_angles[aero::joint::r_shoulder_r] = r_shoulder_r_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendAngleVector(3000);


  ROS_INFO("--- SHOULDER YAW JOINTS ---");
  joint_angles[aero::joint::l_shoulder_y] = l_shoulder_y_to;
  joint_angles[aero::joint::r_shoulder_y] = r_shoulder_y_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendAngleVector(3000);

  ROS_INFO("--- RESET MANIP POSE ---");
  robot->setPoseVariables(aero::pose::reset_manip);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendAngleVector(3000);
  sleep(3);

  // test wrist and neck joints and hand
  double l_wrist_r_to = -0.5;
  double r_wrist_r_to =  0.5;
  double l_wrist_y_to = -0.5;
  double r_wrist_y_to =  0.5;

  robot->getRobotStateVariables(joint_angles);

  ROS_INFO("--- WRIST ROLL JOINTS ---");
  joint_angles[aero::joint::l_wrist_r] = l_wrist_r_to;
  joint_angles[aero::joint::r_wrist_r] = r_wrist_r_to;
  ALLPJOINT();
  ALLSJOINT();
  robot->sendAngleVector(2000);

  ROS_INFO("--- WRIST YAW JOINTS ---");
  joint_angles[aero::joint::l_wrist_y] = l_wrist_y_to;
  joint_angles[aero::joint::r_wrist_y] = r_wrist_y_to;
  robot->setRobotStateVariables(joint_angles);
  ALLPJOINT();
  ALLSJOINT();
  robot->sendAngleVector(2000);

  //ros::shutdown();
  //exit(0);

  ROS_INFO("--- looking at left hand ---");
  robot->setLookAt(robot->getEEFPosition(aero::arm::larm, aero::eef::pick));
  robot->sendAngleVector(1000);

  ROS_INFO("--- testing left hand ---");
  robot->sendGrasp(aero::arm::larm);
  robot->openHand(aero::arm::larm);
  robot->sendHand(aero::arm::larm, 0.0);


  ROS_INFO("--- looking at right hand ---");
  robot->setLookAt(robot->getEEFPosition(aero::arm::rarm, aero::eef::pick));
  robot->sendAngleVector(1000);


  ROS_INFO("--- testing right hand ---");
  robot->sendGrasp(aero::arm::rarm);
  robot->openHand(aero::arm::rarm);
  robot->sendHand(aero::arm::rarm, 0.0);


  robot->resetLookAt();
  ROS_INFO("--- reseting robot pose ---");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(1000);
  sleep(1);


  // test waist
  double waist_y_l_to =  0.7;
  double waist_y_r_to = -0.7;
  double waist_p_to = 0.3;

  robot->getRobotStateVariables(joint_angles);

  ROS_INFO("--- WAIST YAW JOINT ---");
  joint_angles[aero::joint::waist_y] = waist_y_l_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(2000);
  joint_angles[aero::joint::waist_y] = waist_y_r_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(3000);
  joint_angles[aero::joint::waist_y] = 0.0;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(2000);

  ROS_INFO("--- WAIST PITCH JOINT ---");
  joint_angles[aero::joint::waist_p] = waist_p_to;
  robot->setRobotStateVariables(joint_angles);
  robot->sendAngleVector(2000);

  ROS_INFO("--- RESET MANIP POSE ---");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(2000);
  sleep(2);


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
