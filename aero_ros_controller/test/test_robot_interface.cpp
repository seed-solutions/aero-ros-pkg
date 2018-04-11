#include <ros/ros.h>
#include <gtest/gtest.h>

#include <aero_ros_controller/RobotInterface.hh>

using namespace robot_interface;

class AeroRobotInterface : public robot_interface::RobotInterface {
public:
  typedef boost::shared_ptr<AeroRobotInterface > Ptr;

public:
  AeroRobotInterface(ros::NodeHandle &_nh) : robot_interface::RobotInterface(_nh) {
    //
    configureFromParam("robot_interface_controllers");

    //
#define ADD_CONTROLLER(limb)                    \
    {                                           \
      auto it = controllers_.find(#limb);       \
      if (it != controllers_.end()) {           \
        this-> limb = it->second;               \
      }                                         \
    }

    ADD_CONTROLLER(rarm);
    ADD_CONTROLLER(larm);
    ADD_CONTROLLER(waist);
    ADD_CONTROLLER(lifter);
    ADD_CONTROLLER(head);

    add_group("both_arms",   {"rarm", "larm"});
    add_group("upper_body",  {"rarm", "larm", "waist", "head"});
    add_group("torso",       {"waist", "lifter"});
    add_group("without_head",{"rarm", "larm", "waist", "lifter"});

    for(int i = 0; i < joint_list_.size(); i++) {
      ROS_INFO("j%d: %s", i, joint_list_[i].c_str());
    }
  }

public:
  robot_interface::TrajectoryClient::Ptr larm;  // 8
  robot_interface::TrajectoryClient::Ptr rarm;  // 8
  robot_interface::TrajectoryClient::Ptr waist; // 3
  robot_interface::TrajectoryClient::Ptr lifter;// 2
  robot_interface::TrajectoryClient::Ptr head;  // 3 // 24 jsk / 22 thk
  // TrajectoryClient::Ptr l_hand;
  // TrajectoryClient::Ptr r_hand;
};


class RobotInterfaceTest: public testing::Test
{
protected:
  virtual void SetUp()
  {
    //nh.reset(new ros::NodeHandle());
    ros::NodeHandle nh;

    ROS_INFO("Create Interface");
    ari.reset(new AeroRobotInterface(nh));
    //ari = new AeroRobotInterface(*nh);
    ROS_INFO("Create Interface: done");

    a_map["r_shoulder_p_joint"] = -0.18800;
    a_map["r_shoulder_r_joint"] = -0.39268;
    a_map["r_shoulder_y_joint"] = 0.98168;
    a_map["r_elbow_joint"]      = -0.61062;
    a_map["r_wrist_y_joint"]    = 0.78535;
    //a_map["r_wrist_p_joint"]    = 0.07500;
    a_map["r_wrist_p_joint"]    = 0.0;
    a_map["r_wrist_r_joint"]    = 0.80722;
    a_map["r_hand_y_joint"]     = 1.00000;
    a_map["l_shoulder_p_joint"] = -0.18800;
    a_map["l_shoulder_r_joint"] = 1.17803;
    a_map["l_shoulder_y_joint"] = 0.19637;
    a_map["l_elbow_joint"]      = -0.61062;
    a_map["l_wrist_y_joint"]    = 0.78535;
    //a_map["l_wrist_p_joint"]    = 0.07500;
    a_map["l_wrist_p_joint"]    = 0.0;
    a_map["l_wrist_r_joint"]    = 0.19635;
    a_map["l_hand_y_joint"]     = 1.00000;
    a_map["waist_y_joint"]      = 1.00000;
    a_map["waist_p_joint"]      = 0.37083;
    //a_map["waist_r_joint"]      = 0.00850;
    a_map["waist_r_joint"]      = 0.0;
    a_map["ankle_joint"]        = 1.17803;
    a_map["knee_joint"]         = -0.39268;
    a_map["neck_y_joint"]       = 1.05000;
    a_map["neck_p_joint"]       = 0.45815;
    //a_map["neck_r_joint"]       = 0.06109;
    a_map["neck_r_joint"]       = 0.0;

    b_map["r_shoulder_p_joint"] = -1.08600;
    b_map["r_shoulder_r_joint"] = -1.17803;
    b_map["r_shoulder_y_joint"] = -0.19637;
    b_map["r_elbow_joint"]      = -1.83185;
    b_map["r_wrist_y_joint"]    = -0.78535;
    //b_map["r_wrist_p_joint"]    = -0.07500;
    b_map["r_wrist_p_joint"]    =  0.0;
    b_map["r_wrist_r_joint"]    = -0.19635;
    b_map["r_hand_y_joint"]     = -1.00000;
    b_map["l_shoulder_p_joint"] = -1.08600;
    b_map["l_shoulder_r_joint"] = 0.39268;
    b_map["l_shoulder_y_joint"] = -0.98168;
    b_map["l_elbow_joint"]      = -1.83185;
    b_map["l_wrist_y_joint"]    = -0.78535;
    //b_map["l_wrist_p_joint"]    = -0.07500;
    b_map["l_wrist_p_joint"]    =  0.0;
    b_map["l_wrist_r_joint"]    = -0.80722;
    b_map["l_hand_y_joint"]     = -1.00000;
    b_map["waist_y_joint"]      = -1.00000;
    b_map["waist_p_joint"]      = 0.06548;
    //b_map["waist_r_joint"]      = -0.00850;
    b_map["waist_r_joint"]      =  0.0;
    b_map["ankle_joint"]        = 0.39268;
    b_map["knee_joint"]         = -1.17803;
    b_map["neck_y_joint"]       = -1.05000;
    b_map["neck_p_joint"]       = 0.15272;
    //b_map["neck_r_joint"]       = -0.06109;
    b_map["neck_r_joint"]       =  0.0;
  }

  virtual void TearDown() {
    ROS_INFO("Tear Down >>");
    // delete(ari);
    ROS_INFO("Tear Down <<");
  }

  double angle_vector_diff(robot_interface::angle_vector &src,
                           robot_interface::angle_vector &dst)
  {
    int sz = std::min(src.size(), dst.size());
    double diff = 0;
    for(int i = 0; i < sz; i++) {
      double tmp = src[i] - dst[i];
      diff += (tmp * tmp);
    }
    return diff;
  }

  //boost::shared_ptr<ros::NodeHandle > nh;
  //AeroRobotInterface *ari;
  AeroRobotInterface::Ptr ari;

  robot_interface::joint_angle_map a_map;
  robot_interface::joint_angle_map b_map;
};

#define CHECK_RUN_TIME(_func, _tm, _eps)                                \
  {                                                                     \
    ros::Time st = ros::Time::now();                                    \
    _func ;                                                             \
    ros::Time ed = ros::Time::now();                                    \
    double msec = (ed - st).toSec() * 1000;                             \
    ROS_INFO_STREAM(#_func << " takes " << msec << " [ms]");            \
    EXPECT_NEAR(msec, _tm, _eps);                                       \
  }

#define AV_DIFF(src, dst)                       \
  {                                             \
    int sz = std::min(src.size(), dst.size());  \
    double diff = 0;                            \
    for(int i = 0; i < sz; i++) {               \
      double tmp = src[i] - dst[i];             \
      diff += (tmp * tmp);                      \
    }                                           \
  }

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

TEST_F(RobotInterfaceTest, testWait)
{
  robot_interface::angle_vector a_av, b_av;
  ari->convertToAngleVector(a_map, a_av);
  ari->convertToAngleVector(b_map, b_av);

  // check initial wait
  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation("larm"), 0.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation("upper_body"), 0.0, 100.0);
  EXPECT_FALSE(ari->interpolatingp());
  CHECK_RUN_TIME(ari->interpolatingp(), 0.0, 100.0);

  // send whole joints
  ari->send_angle_vector(a_av, 4.0);
  //EXPECT_TRUE(ari->interpolatingp());
  EXPECT_FALSE(ari->wait_interpolation(0.001));
  EXPECT_TRUE(ari->interpolatingp());
  CHECK_RUN_TIME(ari->interpolatingp(), 0.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 4000.0, 150.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 100.0);
  {
    robot_interface::angle_vector tmp;
    ari->potentio_vector(tmp);
    double diff = angle_vector_diff(a_av, tmp);
    EXPECT_NEAR(0, diff, 0.01);
  }

  // send only head
  ari->send_angle_vector(b_av, 5.0, "head");
  CHECK_RUN_TIME(ari->wait_interpolation("larm"), 0.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation("head"), 5100.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 100.0);
  {
    robot_interface::angle_vector src;
    robot_interface::angle_vector dst;
    ari->head->potentio_vector(src);
    ari->head->convertToAngleVector(b_map, dst);
    double diff = angle_vector_diff(src, dst);
#if 0
    ROS_INFO("%ld %ld", src.size(), dst.size());
    for(int i = 0; i < src.size(); i++) {
      ROS_INFO("%d: %f", i, src[i]);
    }
    for(int i = 0; i < dst.size(); i++) {
      ROS_INFO("%d: %f", i, dst[i]);
    }
#endif
    EXPECT_NEAR(0, diff, 0.005);
  }

  //
  ari->send_angle_vector(a_av, 3.0, "upper_body");
  CHECK_RUN_TIME(ari->wait_interpolation("lifter"), 0.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation("larm"), 3100.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation("larm"), 0.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation("upper_body"), 0.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation("lifter"), 0.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 100.0);
}

TEST_F(RobotInterfaceTest, testStopMotion)
{

  robot_interface::angle_vector a_av, b_av;
  ari->convertToAngleVector(a_map, a_av);
  ari->convertToAngleVector(b_map, b_av);

  // send whole joints
  ari->send_angle_vector(a_av, 2.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 2100.0, 100.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 0.0, 100.0);

  //
  ari->send_angle_vector(b_av, 4.0);
  CHECK_RUN_TIME(ari->wait_interpolation(2.0), 2100.0, 100.0);

  ari->stop_motion();
  CHECK_RUN_TIME(ari->wait_interpolation(), 100.0, 100.0);
  { // check not moving
    robot_interface::angle_vector src;
    robot_interface::angle_vector dst;
    ari->potentio_vector(src);
    usleep(5000 * 1000);
    ari->potentio_vector(dst);

    double diff = angle_vector_diff(src, dst);
    EXPECT_NEAR(0, diff, 0.01);
  }

  { // check stopping between a_av and b_av
    robot_interface::angle_vector src;
    ari->potentio_vector(src);

    double diff_a = angle_vector_diff(src, b_av);
    double diff_b = angle_vector_diff(src, a_av);

    ROS_INFO("diff_a = %f, diff_b = %f", diff_a, diff_b);

    EXPECT_TRUE(diff_a > 0.5);
    EXPECT_TRUE(diff_b > 0.5);
  }

  //
  ari->send_angle_vector(a_av, 2.0);
  CHECK_RUN_TIME(ari->wait_interpolation(), 2100.0, 100.0);

  //
  ari->send_angle_vector(b_av, 4.0);
  CHECK_RUN_TIME(ari->wait_interpolation(2.0), 2100.0, 100.0);

  ari->cancel_angle_vector(false);
  CHECK_RUN_TIME(ari->wait_interpolation(), 100.0, 100.0);
  { // check not moving
    robot_interface::angle_vector src;
    robot_interface::angle_vector dst;
    ari->potentio_vector(src);
    usleep(5000 * 1000);
    ari->potentio_vector(dst);

    double diff = angle_vector_diff(src, dst);
    EXPECT_NEAR(0, diff, 0.01);
  }

  { // check stopping between a_av and b_av
    robot_interface::angle_vector src;
    ari->potentio_vector(src);

    double diff_a = angle_vector_diff(src, b_av);
    double diff_b = angle_vector_diff(src, a_av);

    ROS_INFO("diff_a = %f, diff_b = %f", diff_a, diff_b);

    EXPECT_TRUE(diff_a > 0.5);
    EXPECT_TRUE(diff_b > 0.5);
  }
}

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
#define EPS 0.025
  /// move to A
  ari->send_angle_vector(a_av, 5.0);
  ari->wait_interpolation();
  ROS_INFO("move to A");
  robot_interface::angle_vector a_act_av, a_ref_av;
  ari->reference_vector(a_ref_av);
  ari->potentio_vector(a_act_av);
  for(int i = 0; i < prev_act_av.size(); i++) {
    ROS_INFO("%d %f %f %f (%f %f)", i, a_av[i], a_act_av[i], a_ref_av[i],
             std::fabs(a_av[i] - a_act_av[i]),
             std::fabs(a_av[i] - a_ref_av[i]));
    EXPECT_NEAR( a_av[i], a_act_av[i], EPS);
    EXPECT_NEAR( a_av[i], a_ref_av[i], EPS);
  }

  /// move to B
  ari->send_angle_vector(b_av, 8.0);
  ari->wait_interpolation();
  ROS_INFO("move to B");
  robot_interface::angle_vector b_act_av, b_ref_av;
  ari->reference_vector(b_ref_av);
  ari->potentio_vector(b_act_av);
  for(int i = 0; i < b_act_av.size(); i++) {
    ROS_INFO("%d %f %f %f (%f %f)", i, b_av[i], b_act_av[i], b_ref_av[i],
             std::fabs(b_av[i] - b_act_av[i]),
             std::fabs(b_av[i] - b_ref_av[i]));
    EXPECT_NEAR( b_av[i], b_act_av[i], EPS);
    EXPECT_NEAR( b_av[i], b_ref_av[i], EPS);
  }

  /// move to A (only rarm)
  ari->send_angle_vector(a_av, 8.0, "rarm");
  ari->wait_interpolation();

  ROS_INFO("move to A (only rarm)");
  robot_interface::angle_vector curr_act_av, curr_ref_av;
  ari->reference_vector(curr_ref_av);
  ari->potentio_vector(curr_act_av);
  for(int i = 0; i < 7; i++) {
    ROS_INFO("%d %f %f %f (%f %f)", i, a_av[i], curr_act_av[i], curr_ref_av[i],
             std::fabs(a_av[i] - curr_act_av[i]),
             std::fabs(a_av[i] - curr_ref_av[i]));
    EXPECT_NEAR( a_av[i], curr_act_av[i], EPS);
    EXPECT_NEAR( a_av[i], curr_ref_av[i], EPS);
  }
  for(int i = 7; i < curr_act_av.size(); i++) {
    ROS_INFO("%d %f %f %f (%f %f)", i, b_av[i], curr_act_av[i], curr_ref_av[i],
             std::fabs(b_av[i] - curr_act_av[i]),
             std::fabs(b_av[i] - curr_ref_av[i]));
    EXPECT_NEAR( b_av[i], curr_act_av[i], EPS);
    EXPECT_NEAR( b_av[i], curr_ref_av[i], EPS);
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
send angle-vector-sequence

start-time-check

send small-joint-map
 */
