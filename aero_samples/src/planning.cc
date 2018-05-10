#include <aero_std/AeroMotionPlanningInterface.hh>

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "planning_sample_node");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  aero::interface::AeroMotionPlanningInterface::Ptr planning
    (new aero::interface::AeroMotionPlanningInterface(nh, robot->kinematic_model));

  // reset robot
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->setLifter(0, -0.38);
  robot->setHand(aero::arm::rarm, 0.9);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  // set start state
  planning->setCurrentState(robot);

  // add collision obstacles
  planning->processCollisionMesh
    ("mesh", "base_link", aero::Translation(0.6, 0.0, 0.7) * aero::Quaternion(1.0, 0.0, 0.0, 0.0),
     "package://aero_samples/models/container.dae");

  // setup goal state and plan settings
  Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(45.0*M_PI/180.0, Eigen::Vector3d::UnitY())))
    * Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(-90.0*M_PI/180.0, Eigen::Vector3d::UnitX())));
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = aero::moveGroup(aero::arm::rarm, aero::ikrange::upperbody);
  std::string eef_link = aero::eefLink(aero::arm::rarm, aero::eef::grasp);
  moveit_msgs::Constraints pose_goal = planning->constructGoalConstraintsFromIK
    (robot, req.group_name, aero::Translation(0.6, -0.2, 0.8) * qua, eef_link, 0.03, 0.03);
  req.goal_constraints.push_back(pose_goal);
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -2.0; req.workspace_parameters.min_corner.z = 0.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 2.0; req.workspace_parameters.max_corner.z = 10.0;
  req.planner_id = "RRTstarkConfigDefault";
  req.allowed_planning_time = 1.0;

  // if (!planning->solve(req, res))
  //   return 0;

  planning->solveEEFCollisionEnabled(req, res, 1);

  // visualization

  planning->displayTrajectory(res);

  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();

  ROS_INFO("demo node finished");
  ros::shutdown();

  return 0;
}
