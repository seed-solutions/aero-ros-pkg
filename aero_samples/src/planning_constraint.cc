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

  // set initial robot pose
  ROS_INFO("initial robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->setLifter(0, -0.2);
  planning_interface::MotionPlanRequest req;
  req.group_name = aero::moveGroup(aero::arm::rarm, aero::ikrange::wholebody);
  std::string eef_link = aero::eefLink(aero::arm::rarm, aero::eef::grasp);
  Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(-90.0*M_PI/180.0, Eigen::Vector3d::UnitX())));
  robot->setFromIK(req.group_name, aero::Translation(0.8, -0.2, 1.0) * qua, eef_link);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  // set start state
  planning->setCurrentState(robot);

  // setup goal state and plan settings
  planning_interface::MotionPlanResponse res;
  moveit_msgs::Constraints pose_goal = planning->constructGoalConstraintsFromIK
    (robot, req.group_name, aero::Translation(0.4, -0.2, 1.0) * qua, eef_link, 0.03, 0.03);
  req.goal_constraints.push_back(pose_goal);
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -2.0; req.workspace_parameters.min_corner.z = 0.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 2.0; req.workspace_parameters.max_corner.z = 10.0;
  req.planner_id = "RRTstarkConfigDefault";
  req.allowed_planning_time = 60.0;

  // add path constraint
  req.path_constraints = planning->constructPathConstraintsFromQuaternion(eef_link, "base_link", qua);

  if (!planning->solve(req, res))
    return 0;

  // visualization

  planning->displayTrajectory(res);

  ros::WallDuration sleep_time(15.0);
  sleep_time.sleep();

  ROS_INFO("demo node finished");
  ros::shutdown();

  return 0;
}
