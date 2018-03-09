#include <ros/ros.h>
#include <aero_ros_controller/RobotInterface.hh>

#include <Eigen/Eigen>

static std::ostream& operator<<(std::ostream& os, const Eigen::Affine3d &tr)
{
  Eigen::Vector3d tt = tr.translation();
  Eigen::Quaterniond qq(tr.linear());
  os << "(cons #f("
     << tt(0) << " "
     << tt(1) << " "
     << tt(2) << ")";
  os << " #f("
     << qq.w() << " "
     << qq.x() << " "
     << qq.y() << " "
     << qq.z() << "))";
  return os;
}

using namespace robot_interface;

//// TEST for aero TypeF
class AeroRobotInterface : public RobotInterface {
public:
  typedef boost::shared_ptr<AeroRobotInterface > Ptr;

public:
  AeroRobotInterface(ros::NodeHandle &_nh) : RobotInterface(_nh) {
    // rarm
    rarm.reset(new TrajectoryClient(_nh,
                                    "rarm_controller/follow_joint_trajectory",
                                    "rarm_controller/state",
                                    { "r_shoulder_p_joint", "r_shoulder_r_joint", "r_shoulder_y_joint",
                                        "r_elbow_joint", "r_wrist_y_joint", "r_wrist_p_joint", "r_wrist_r_joint",
                                        // "r_hand_y_joint", // jsk
                                        }
                                    ));
    this->add_controller("rarm", rarm);

    // larm
    larm.reset(new TrajectoryClient(_nh,
                                    "larm_controller/follow_joint_trajectory",
                                    "larm_controller/state",
                                    { "l_shoulder_p_joint", "l_shoulder_r_joint", "l_shoulder_y_joint",
                                        "l_elbow_joint", "l_wrist_y_joint", "l_wrist_p_joint", "l_wrist_r_joint",
                                        // "l_hand_y_joint", // jsk
                                        }
                                    ));
    this->add_controller("larm", larm);

    // torso
    waist.reset(new TrajectoryClient(_nh,
                                     "waist_controller/follow_joint_trajectory",
                                     "waist_controller/state",
                                     { "waist_y_joint", "waist_p_joint", "waist_r_joint"}
                                     ));
    this->add_controller("waist",  waist);

    // lifter
    lifter.reset(new TrajectoryClient(_nh,
                                      "lifter_controller/follow_joint_trajectory",
                                      "lifter_controller/state",
                                      { "knee_joint", "ankle_joint" }
                                      ));
    this->add_controller("lifter", lifter);

    // head
    head.reset(new TrajectoryClient(_nh,
                                    "head_controller/follow_joint_trajectory",
                                    "head_controller/state",
                                    { "neck_y_joint", "neck_p_joint", "neck_r_joint"}
                                    ));
    this->add_controller("head",   head);

    // group settings
    controller_group_["both_arms"]  = {"rarm", "larm"};
    controller_group_["upper_body"] = {"rarm", "larm", "waist", "head"};
    controller_group_["torso"]      = {"waist", "lifter"};
  }

public:
  TrajectoryClient::Ptr larm; // 8
  TrajectoryClient::Ptr rarm; // 8
  TrajectoryClient::Ptr waist; // 3
  TrajectoryClient::Ptr lifter;// 2
  TrajectoryClient::Ptr head;  // 3 // 24 jsk / 22 thk
  // TrajectoryClient::Ptr l_hand;
  // TrajectoryClient::Ptr r_hand;
};

int main (int argc, char **argv) {
  ros::init(argc, argv, "robot_interface_test");
  ros::NodeHandle nh;

  ROS_INFO("Create Interface");
  AeroRobotInterface::Ptr ari(new AeroRobotInterface(nh));
  ROS_INFO("Create Interface: done");

  robot_interface::joint_angle_map a_map;
  robot_interface::joint_angle_map b_map;
  a_map["r_shoulder_p_joint"] = -0.18800;
  a_map["r_shoulder_r_joint"] = -0.39268;
  a_map["r_shoulder_y_joint"] = 0.98168;
  a_map["r_elbow_joint"]      = -0.61062;
  a_map["r_wrist_y_joint"]    = 0.78535;
  a_map["r_wrist_p_joint"]    = 0.07500;
  a_map["r_wrist_r_joint"]    = 0.80722;
  a_map["r_hand_y_joint"]     = 1.00000;
  a_map["l_shoulder_p_joint"] = -0.18800;
  a_map["l_shoulder_r_joint"] = 1.17803;
  a_map["l_shoulder_y_joint"] = 0.19637;
  a_map["l_elbow_joint"]      = -0.61062;
  a_map["l_wrist_y_joint"]    = 0.78535;
  a_map["l_wrist_p_joint"]    = 0.07500;
  a_map["l_wrist_r_joint"]    = 0.19635;
  a_map["l_hand_y_joint"]     = 1.00000;
  a_map["waist_y_joint"]      = 1.00000;
  a_map["waist_p_joint"]      = 0.37083;
  a_map["waist_r_joint"]      = 0.00850;
  a_map["ankle_joint"]        = 1.17803;
  a_map["knee_joint"]         = -0.39268;
  a_map["neck_y_joint"]       = 1.05000;
  a_map["neck_p_joint"]       = 0.45815;
  a_map["neck_r_joint"]       = 0.06109;

  b_map["r_shoulder_p_joint"] = -1.08600;
  b_map["r_shoulder_r_joint"] = -1.17803;
  b_map["r_shoulder_y_joint"] = -0.19637;
  b_map["r_elbow_joint"]      = -1.83185;
  b_map["r_wrist_y_joint"]    = -0.78535;
  b_map["r_wrist_p_joint"]    = -0.07500;
  b_map["r_wrist_r_joint"]    = -0.19635;
  b_map["r_hand_y_joint"]     = -1.00000;
  b_map["l_shoulder_p_joint"] = -1.08600;
  b_map["l_shoulder_r_joint"] = 0.39268;
  b_map["l_shoulder_y_joint"] = -0.98168;
  b_map["l_elbow_joint"]      = -1.83185;
  b_map["l_wrist_y_joint"]    = -0.78535;
  b_map["l_wrist_p_joint"]    = -0.07500;
  b_map["l_wrist_r_joint"]    = -0.80722;
  b_map["l_hand_y_joint"]     = -1.00000;
  b_map["waist_y_joint"]      = -1.00000;
  b_map["waist_p_joint"]      = 0.06548;
  b_map["waist_r_joint"]      = -0.00850;
  b_map["ankle_joint"]        = 0.39268;
  b_map["knee_joint"]         = -1.17803;
  b_map["neck_y_joint"]       = -1.05000;
  b_map["neck_p_joint"]       = 0.15272;
  b_map["neck_r_joint"]       = -0.06109;

#define PRINT_SIZE(_av)\
  ROS_INFO_STREAM(#_av << ".size() =  " <<  _av.size())

  robot_interface::angle_vector a_av, b_av;
  ari->convertToAngleVector(a_map, a_av);
  ari->convertToAngleVector(b_map, b_av);
  PRINT_SIZE(a_av);

  robot_interface::angle_vector l_a_av, l_b_av;
  ari->larm->convertToAngleVector(a_map, l_a_av);
  ari->larm->convertToAngleVector(b_map, l_b_av);
  PRINT_SIZE(l_a_av);

  robot_interface::angle_vector r_a_av, r_b_av;
  ari->rarm->convertToAngleVector(a_map, r_a_av);
  ari->rarm->convertToAngleVector(b_map, r_b_av);
  robot_interface::angle_vector w_a_av, w_b_av;
  robot_interface::angle_vector h_a_av, h_b_av;
  robot_interface::angle_vector f_a_av, f_b_av;
  ari->waist->convertToAngleVector(a_map, w_a_av);
  ari->waist->convertToAngleVector(b_map, w_b_av);
  ari->head->convertToAngleVector(a_map, h_a_av);
  ari->head->convertToAngleVector(b_map, h_b_av);
  ari->lifter->convertToAngleVector(a_map, f_a_av);
  ari->lifter->convertToAngleVector(b_map, f_b_av);
  PRINT_SIZE(w_a_av);
  PRINT_SIZE(h_a_av);
  PRINT_SIZE(f_a_av);

#define PRINT_RUN_TIME(_func)\
  {                                                                     \
    ros::Time st = ros::Time::now();                                    \
    _func ;                                                             \
    ros::Time ed = ros::Time::now();                                    \
    ROS_INFO_STREAM(#_func << " takes " << (ed - st).toSec() * 1000 << " [ms]"); \
  }
  PRINT_RUN_TIME(ari->wait_interpolation());
  PRINT_RUN_TIME(ari->wait_interpolation("larm"));
  PRINT_RUN_TIME(ari->wait_interpolation("upper_body"));

  {
    robot_interface::joint_angle_map r_map, a_map;
    robot_interface::angle_vector r_av, a_av;
    ari->head->getReferencePositions(r_map);
    ari->head->getActualPositions(a_map);
    ari->head->reference_vector(r_av);
    ari->head->potentio_vector(a_av);
    PRINT_SIZE(r_map);
    PRINT_SIZE(a_map);
    PRINT_SIZE(r_av);
    PRINT_SIZE(a_av);
  }

  {
    robot_interface::joint_angle_map r_map, a_map;
    robot_interface::angle_vector r_av, a_av;
    ari->getReferencePositions(r_map);
    ari->getActualPositions(a_map);
    ari->reference_vector(r_av);
    ari->potentio_vector(a_av);
#if 0
    for(auto it = r_map.begin(); it != r_map.end(); it++) {
      ROS_INFO("%s : %f", it->first.c_str(), it->second);
    }
#endif
    PRINT_SIZE(r_map);
    PRINT_SIZE(a_map);
    PRINT_SIZE(r_av);
    PRINT_SIZE(a_av);
  }

  ROS_INFO("send larm angle-vector");
  ari->larm->send_angle_vector(l_a_av, 5.0);
  PRINT_RUN_TIME(ari->larm->wait_interpolation(2.0));
  {
    robot_interface::angle_vector r_av, a_av;
    ari->larm->reference_vector(r_av);
    ari->larm->potentio_vector(a_av);
    PRINT_SIZE(r_av);
    PRINT_SIZE(a_av);
    ROS_INFO("%f %f / %f - %f",
             r_av[0], a_av[0],
             l_a_av[0], l_b_av[0]);
  }
  PRINT_RUN_TIME(ari->larm->wait_interpolation());
  PRINT_RUN_TIME(ari->larm->wait_interpolation());

  {
    robot_interface::angle_vector r_av, a_av;
    ari->larm->reference_vector(r_av);
    ari->larm->potentio_vector(a_av);
    PRINT_SIZE(r_av);
    PRINT_SIZE(a_av);
    ROS_INFO("%f %f / %f - %f",
             r_av[0], a_av[0],
             l_a_av[0], l_b_av[0]);
  }

  ROS_INFO("send larm angle-vector / stop");
  ari->larm->send_angle_vector(l_b_av, 5.0);
  PRINT_RUN_TIME(ari->larm->wait_interpolation(2.0));
  {
    robot_interface::angle_vector r_av, a_av;
    ari->larm->reference_vector(r_av);
    ari->larm->potentio_vector(a_av);
    PRINT_SIZE(r_av);
    PRINT_SIZE(a_av);
    ROS_INFO("%f %f / %f - %f",
             r_av[0], a_av[0],
             l_a_av[0], l_b_av[0]);
  }
  // stop_motion
  {
    robot_interface::joint_angle_map _map;
    ari->larm->getReferencePositions(_map);
    //ari.larm->getActualVector(_map);
    ros::Time start_ = ros::Time::now() + ros::Duration(0.0);
    ari->larm->sendAngles(_map, 0.1, start_);
  }
  PRINT_RUN_TIME(ari->larm->wait_interpolation());
  PRINT_RUN_TIME(ari->larm->wait_interpolation());
  {
    robot_interface::angle_vector r_av, a_av;
    ari->larm->reference_vector(r_av);
    ari->larm->potentio_vector(a_av);
    PRINT_SIZE(r_av);
    PRINT_SIZE(a_av);
    ROS_INFO("%f %f / %f - %f",
             r_av[0], a_av[0],
             l_a_av[0], l_b_av[0]);
  }

  {
    std::vector< std::string> nms = {"larm" };
    ari->send_angle_vector(b_av, 5.0, nms);
    ari->send_angle_vector(b_av, 2.0, "rarm");
    //ari->send_angle_vector(b_av, 5.0);
    PRINT_RUN_TIME(ari->wait_interpolation("rarm"));
    PRINT_RUN_TIME(ari->wait_interpolation("head"));
    PRINT_RUN_TIME(ari->wait_interpolation("waist"));
    PRINT_RUN_TIME(ari->wait_interpolation("upper_body"));
    PRINT_RUN_TIME(ari->wait_interpolation());
  }

#if 0
  ////
  ROS_INFO("wait interpolation");
  bool ret = ari.wait_interpolation();
  ari.wait_interpolation("larm");
  ari.wait_interpolation("larm", 1.0);
  std::vector < std::string> groups = {"larm", "rarm", "waist"};
  ari.wait_interpolation(groups, 2.0);

  ari.wait_interpolation("both_arms", 2.0);

  ROS_INFO("ari.wait_interpolation() => %d", ret);

  robot_interface::joint_angle_map ref_map, act_map;
  ari.getReferencePositions(ref_map);
  ari.getActualPositions(ref_map);
  robot_interface::angle_vector ref_av, act_av;
  ari.reference_vector(ref_av);
  ari.reference_vector(act_av);
#endif

#if 0
  ari.send_angle_vector();

  ari.send_angle_vector_sequence();

  ari.larm.send_angle_vector();

  ari.larm.send_angle_vector_sequence();
#endif

#if 0
  ros::Duration d(10);
  if( ari.rarm->waitForResult(d)) {
    ROS_INFO("success");
    actionlib::SimpleClientGoalState state = ari.rarm->getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("fail");
  }
#endif
  // ros::spin();
}
