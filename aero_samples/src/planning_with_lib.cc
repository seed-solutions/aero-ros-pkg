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
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = aero::moveGroup(aero::arm::rarm, aero::ikrange::upperbody);
  // goal must be in handLink
  Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(45.0*M_PI/180.0, Eigen::Vector3d::UnitY())))
    * Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(-90.0*M_PI/180.0, Eigen::Vector3d::UnitX())));
  aero::Transform transform = planning->getTransformInMoveGroupEEF
    (robot, req.group_name, aero::arm::rarm, aero::eef::grasp, aero::Translation(0.6, -0.2, 0.8) * qua);
  // set goal pose
  moveit_msgs::Constraints pose_goal = planning->constructGoalConstraints("base_link", transform, aero::arm::rarm, 0.03, 0.03);
  req.goal_constraints.push_back(pose_goal);
  // set planner settings
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -2.0; req.workspace_parameters.min_corner.z = 0.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 2.0; req.workspace_parameters.max_corner.z = 10.0;
  req.planner_id = "BKPIECEkConfigDefault"; //"RRTstarkConfigDefault";
  req.allowed_planning_time = 2.0;

  // visualize scene

  planning->displayScene();

  // try solve ten times

  for (int i = 0; i < 10; ++i) {
    ROS_INFO("running %d", i);
    if (planning->plan(req, res)) {
      // visualize path and collision if any
      planning->checkCollision(robot, res);
      planning->displayTrajectory(res);
      planning->execute(robot, res, 3000, aero::ikrange::upperbody);
      // wait trajectory finish
      robot->waitInterpolation();
      // reset pose
      robot->setPoseVariables(aero::pose::reset_manip);
      robot->sendModelAngles(5000);
      robot->waitInterpolation();
    }
    // ros::WallDuration sleep_time(5.0);
    // sleep_time.sleep();
  }

  planning->displayScene(); // should show finished state

  ROS_INFO("demo node finished");
  ros::shutdown();

  return 0;
}
