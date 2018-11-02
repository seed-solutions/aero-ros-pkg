#include <ros/ros.h>
#include <gtest/gtest.h>

#include <aero_std/AeroMoveitInterface.hh>

class IKTest: public testing::Test
{
protected:
  virtual void SetUp()
  {
    ros::NodeHandle nh;

    ROS_INFO("Create Interface");
    robot.reset(new aero::interface::AeroMoveitInterface(nh));

    robot->setPoseVariables(aero::pose::reset_manip);
    robot->sendModelAngles(3000);
    robot->waitInterpolation();

    ROS_INFO("default pose");
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

  aero::interface::AeroMoveitInterface::Ptr robot;
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

TEST_F(IKTest, testSolveIK)
{
  aero::Translation pos(0.57, -0.15, 1.1);
  aero::Quaternion  rot(1.0, 0.0, 0.0, 0.0);
  aero::Transform   pose1 = pos * rot;

  ROS_INFO_STREAM("ik target: " << pose1);

#define TEST_IK_TIMES 200
  int suc_arm = 0;
  int suc_wb = 0;
  double max_arm_pos_diff = -100000;
  double max_arm_rot_diff = -100000;
  double max_wb_pos_diff = -100000;
  double max_wb_rot_diff = -100000;

  for(int i = 0; i < TEST_IK_TIMES; i++) {
    robot->setPoseVariables(aero::pose::reset_manip);
    robot->updateLinkTransforms();
    bool ik_result_arm = robot->setFromIK(aero::arm::rarm, aero::ikrange::arm, pose1,
                                       aero::eef::grasp);
    if (ik_result_arm) {
      aero::Transform trans;
      robot->getEndEffectorCoords(aero::arm::rarm, aero::eef::grasp, trans);
      aero::Transform diff  = trans.inverse() * pose1;
      max_arm_pos_diff = std::max(std::abs(diff.translation().norm()), max_arm_pos_diff);
      max_arm_rot_diff = std::max(std::abs(aero::AngleAxis(diff.linear()).angle()), max_arm_rot_diff);
    }

    robot->setPoseVariables(aero::pose::reset_manip);
    robot->updateLinkTransforms();
    bool ik_result_wb = robot->setFromIK(aero::arm::rarm, aero::ikrange::wholebody, pose1,
                                       aero::eef::grasp);
    if (ik_result_wb) {
      aero::Transform trans;
      robot->getEndEffectorCoords(aero::arm::rarm, aero::eef::grasp, trans);
      aero::Transform diff  = trans.inverse() * pose1;
      max_wb_pos_diff = std::max(std::abs(diff.translation().norm()), max_wb_pos_diff);
      max_wb_rot_diff = std::max(std::abs(aero::AngleAxis(diff.linear()).angle()), max_wb_rot_diff);
    }

    if (ik_result_arm) suc_arm++;
    if (ik_result_wb)  suc_wb++;
    // ROS_INFO("ik %d %d", ik_result_arm, ik_result_wb);
  }

  ROS_INFO("success rate: arm %f \%, whole_body %f \%",
           100*suc_arm/float(TEST_IK_TIMES),
           100*suc_wb/float(TEST_IK_TIMES));
  EXPECT_GE(100*suc_arm/float(TEST_IK_TIMES), 99.0);
  EXPECT_GE(100*suc_wb/float(TEST_IK_TIMES),  99.0);

  ROS_INFO("arm max_diff(pos/rot) %e/%e", max_arm_pos_diff, max_arm_rot_diff);
  ROS_INFO("whole body max_diff(pos/rot) %e/%e", max_wb_pos_diff, max_wb_rot_diff);
  EXPECT_LE(max_arm_pos_diff, 0.001);
  EXPECT_LE(max_wb_pos_diff,  0.001);
  EXPECT_LE(max_arm_rot_diff, 0.001);
  EXPECT_LE(max_wb_rot_diff,  0.001);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_robot_interface");
  ros::NodeHandle nh;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
