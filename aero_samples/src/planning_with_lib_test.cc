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
  robot->sendModelAngles(3000);
  robot->waitInterpolation();

  // set start state
  planning->setCurrentState(robot);

  // setup goal state and plan settings
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  req.group_name = aero::moveGroup(aero::arm::larm, aero::ikrange::upperbody);
  moveit_msgs::Constraints pose_goal1 = planning->constructGoalConstraints("/odom", aero::Translation(0.305, 0.375, 1.205) * aero::Quaternion(std::sqrt(0.5), 0, -std::sqrt(0.5), 0), aero::arm::larm, 0.03, 0.03);
  req.goal_constraints.push_back(pose_goal1);
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y = -2.0; req.workspace_parameters.min_corner.z = 0.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y = 2.0; req.workspace_parameters.max_corner.z = 10.0;
  req.planner_id = "BKPIECEkConfigDefault";
  req.allowed_planning_time = 10.0;

  // solve
  if (planning->plan(req, res)) {
    planning->displayTrajectory(res);
  } else {
    ROS_ERROR("could not solve plan1!");
  }

  // second test

  // add box collision
  planning->processCollisionBox
    ("box1", "base_link", aero::Translation(0.45, 0.0, 1.0) * aero::Quaternion(1.0, 0.0, 0.0, 0.0), aero::Vector3(0.1, 0.4, 0.8));

  // setup goal state
  moveit_msgs::Constraints pose_goal2 = planning->constructGoalConstraints("/odom", aero::Translation(0.631, 0.173, 1.16) * aero::Quaternion(-0.5, 0.5, 0.5, 0.5), aero::arm::larm, 0.03, 0.03);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal2);  

  // visualize scene

  planning->displayScene();

  // try solve ten times

  for (int i = 0; i < 10; ++i) {
    ROS_INFO("%d: plan2", i);
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
  }

  planning->displayScene(); // should show finished state

  ROS_INFO("demo node finished");
  ros::shutdown();

  return 0;
}
