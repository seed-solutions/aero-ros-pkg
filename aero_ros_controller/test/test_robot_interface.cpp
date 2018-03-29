#include <ros/ros.h>
#include <gtest/gtest.h>

#include <aero_ros_controller/RobotInterface.hh>

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

class RobotInterfaceTest: public testing::Test
{
protected:
  virtual void SetUp()
  {
    nh.reset(new ros::NodeHandle());

    ROS_INFO("Create Interface");
    ari.reset(new AeroRobotInterface(*nh));
    ROS_INFO("Create Interface: done");

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
  }

  virtual void TearDown() {
    ROS_INFO("Tear Down");
  }

  boost::shared_ptr< ros::NodeHandle> nh;
  AeroRobotInterface::Ptr ari;

  robot_interface::joint_angle_map a_map;
  robot_interface::joint_angle_map b_map;
};

#if 0
TEST_F(RobotInterfaceTest, testAngleConvert)
{
  robot_interface::angle_vector a_av, b_av;
  ari->convertToAngleVector(a_map, a_av);
  ari->convertToAngleVector(b_map, b_av);

  EXPECT_EQ (a_av.size(), b_av.size());
  EXPECT_EQ (a_av.size(), 22);

  robot_interface::angle_vector l_a_av;
  ari->larm->convertToAngleVector(a_map, l_a_av);
  EXPECT_EQ (l_a_av.size(), 7);

  robot_interface::angle_vector r_a_av;
  ari->rarm->convertToAngleVector(a_map, r_a_av);
  EXPECT_EQ (r_a_av.size(), 7);

  robot_interface::angle_vector w_a_av;
  ari->waist->convertToAngleVector(a_map, w_a_av);
  EXPECT_EQ (w_a_av.size(), 3);

  robot_interface::angle_vector h_a_av;
  ari->waist->convertToAngleVector(a_map, h_a_av);
  EXPECT_EQ (h_a_av.size(), 3);

  robot_interface::angle_vector f_a_av;
  ari->lifter->convertToAngleVector(a_map, f_a_av);
  EXPECT_EQ (f_a_av.size(), 2);

  {
    robot_interface::joint_angle_map map;
    ari->convertToMap(a_av, map);
    EXPECT_EQ (map.size(), 22);
  }

  {
    robot_interface::joint_angle_map map;
    ari->larm->convertToMap(a_av, map);
    EXPECT_EQ (map.size(), 7);
  }
  // EXPECT_FLAOT_EQ (a, b);
}

#define CHECK_RUN_TIME(_func, _tm, _eps)                                \
  {                                                                     \
    ros::Time st = ros::Time::now();                                    \
    _func ;                                                             \
    ros::Time ed = ros::Time::now();                                    \
    double msec = (ed - st).toSec() * 1000;                             \
    ROS_INFO_STREAM(#_func << " takes " << msec << " [ms]");            \
    EXPECT_NEAR(msec, _tm, _eps);                                       \
  }

TEST_F(RobotInterfaceTest, testWait)
{
  robot_interface::angle_vector a_av, b_av;
  ari->convertToAngleVector(a_map, a_av);
  ari->convertToAngleVector(b_map, b_av);

  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 50.0);
  CHECK_RUN_TIME(ari->wait_interpolation("larm"), 0.0, 50.0);
  CHECK_RUN_TIME(ari->wait_interpolation("upper_body"), 0.0, 50.0);

  ari->send_angle_vector(a_av, 3.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 3100.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 50.0);

  ari->send_angle_vector(b_av, 3.0, "head");
  CHECK_RUN_TIME(ari->wait_interpolation("larm"), 0.0, 50.0);
  CHECK_RUN_TIME(ari->wait_interpolation("head"), 3100.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 50.0);

  ari->send_angle_vector(a_av, 3.0, "upper_body");
  CHECK_RUN_TIME(ari->wait_interpolation("lifter"), 0.0, 50.0);
  CHECK_RUN_TIME(ari->wait_interpolation("larm"), 3100.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation("larm"), 0.0, 50.0);
  CHECK_RUN_TIME(ari->wait_interpolation("upper_body"), 0.0, 50.0);
  CHECK_RUN_TIME(ari->wait_interpolation("lifter"), 0.0, 50.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 50.0);
}
#endif

TEST_F(RobotInterfaceTest, testReferenceActual)
{
  robot_interface::angle_vector a_av, b_av;
  ari->convertToAngleVector(a_map, a_av);
  ari->convertToAngleVector(b_map, b_av);

  robot_interface::angle_vector prev_act_av, prev_ref_av;
  ari->reference_vector(prev_ref_av);
  ari->potentio_vector(prev_act_av);
  EXPECT_EQ( prev_act_av.size(), prev_ref_av.size());
  EXPECT_EQ( a_av.size(), prev_ref_av.size());

  /// move to A
  ari->send_angle_vector(a_av, 3.0);
  ari->wait_interpolation();

  robot_interface::angle_vector a_act_av, a_ref_av;
  ari->reference_vector(a_ref_av);
  ari->potentio_vector(a_act_av);
  for(int i = 0; i < prev_act_av.size(); i++) {
    ROS_INFO("%d %f %f %f (%f)", i, a_av[i], a_act_av[i], a_ref_av[i],
             std::fabs(a_av[i] - a_act_av[i]),
             std::fabs(a_av[i] - a_ref_av[i]));
    EXPECT_NEAR( a_av[i], a_act_av[i], 0.05);
    EXPECT_NEAR( a_av[i], a_ref_av[i], 0.05);
  }

  /// move to B
  ari->send_angle_vector(b_av, 3.0);
  ari->wait_interpolation();

  robot_interface::angle_vector b_act_av, b_ref_av;
  ari->reference_vector(a_ref_av);
  ari->potentio_vector(a_act_av);
  for(int i = 0; i < prev_act_av.size(); i++) {
    EXPECT_NEAR( b_av[i], b_act_av[i], 0.05);
    EXPECT_NEAR( b_av[i], b_ref_av[i], 0.05);
  }

  /// move to A (only rarm)
  ari->send_angle_vector(a_av, 3.0, "rarm");
  ari->wait_interpolation();

  robot_interface::angle_vector curr_act_av, curr_ref_av;
  ari->reference_vector(a_ref_av);
  ari->potentio_vector(a_act_av);
  for(int i = 0; i < 7; i++) {
    EXPECT_NEAR( a_av[i], curr_act_av[i], 0.05);
    EXPECT_NEAR( a_av[i], curr_ref_av[i], 0.05);
  }
  for(int i = 7; i < prev_act_av.size(); i++) {
    EXPECT_NEAR( b_av[i], curr_act_av[i], 0.05);
    EXPECT_NEAR( b_av[i], curr_ref_av[i], 0.05);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_robot_interface");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/*

xxikinari
  wait_interpolation

xxsend/wait
  wait_interpolation

xx redundant wait_interpolation

xx reference check

xx actual check

xx multiple/send
xx multiple/wait


xxmultiple/send
xxsingle/wait


xsend angle-vector all
xsend angle-vector controller
xsend angle-vector group

send angle-vector-sequence

start-time-check

send small-joint-map
 */
