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
  std::string eef_link = aero::eefLink(aero::arm::rarm, aero::eef::hand);
  Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(-90.0*M_PI/180.0, Eigen::Vector3d::UnitX())));
  // transform must be in handLink
  aero::Transform transform = planning->getTransformInMoveGroupEEF
    (robot, req.group_name, aero::arm::rarm, aero::eef::grasp, aero::Translation(0.8, -0.2, 1.0) * qua);
  // set start state
  robot->setFromIK(req.group_name, transform, eef_link);
  planning->setCurrentState(robot);

  // setup goal state and plan settings
  planning_interface::MotionPlanResponse res;
  // set goal pose
  transform = planning->getTransformInMoveGroupEEF
    (robot, req.group_name, aero::arm::rarm, aero::eef::grasp, aero::Translation(0.4, -0.2, 1.0) * qua);
  robot->setFromIK(req.group_name, transform, eef_link);
  robot->sendModelAngles(3000);
  robot->waitInterpolation();
  // computing path constraints take time, use joint goals
  moveit_msgs::Constraints pose_goal = planning->constructGoalConstraints(robot, req.group_name, 0.03, 0.03);
  req.goal_constraints.push_back(pose_goal);
  // set planner settings
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -2.0; req.workspace_parameters.min_corner.z = 0.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 2.0; req.workspace_parameters.max_corner.z = 10.0;
  req.planner_id = "RRTstarkConfigDefault";//"BKPIECEkConfigDefault";
  req.allowed_planning_time = 60.0;

  // add path constraint
  req.path_constraints = planning->constructPathConstraints
    (aero::arm::rarm, "base_link", aero::Quaternion(transform.linear()));

  // visualize scene

  planning->displayScene();

  // try solve ten times

  for (int i = 0; i < 3; ++i) {
    ROS_INFO("running %d", i);
    if (planning->plan(req, res)) {
      // visualize path and collision if any
      planning->checkCollision(robot, res);
      planning->displayTrajectory(res);
    }
    ros::WallDuration sleep_time(5.0);
    sleep_time.sleep();
  }

  ROS_INFO("demo node finished");
  ros::shutdown();

  return 0;
}
