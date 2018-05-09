#include "aero_std/AeroMotionPlanningInterface.hh"

aero::interface::AeroMotionPlanningInterface::AeroMotionPlanningInterface
(ros::NodeHandle &_nh, robot_model::RobotModelConstPtr _rm, std::string _plugin) {
  try {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
  } catch (pluginlib::PluginlibException& ex) {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }

  planning_scene.reset(new planning_scene::PlanningScene(_rm));
  const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
  std::stringstream ss;
  for (std::size_t i = 0; i < classes.size(); ++i)
    ss << classes[i] << " ";
  ROS_INFO_STREAM("Available plugins: " << ss.str());

  try {
    planner_manager.reset(planner_plugin_loader->createUnmanagedInstance(_plugin));
    if (!planner_manager->initialize(_rm, _nh.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_manager->getDescription() << "'");
  } catch (pluginlib::PluginlibException& ex) {
    ROS_ERROR_STREAM("Exception while loading planner '" << _plugin << "': " << ex.what() << std::endl);
  }

  display_publisher_ =
    _nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
}

aero::interface::AeroMotionPlanningInterface::~AeroMotionPlanningInterface() {
}

void aero::interface::AeroMotionPlanningInterface::setCurrentState
(aero::interface::AeroMoveitInterface::Ptr _robot) {  
  _robot->updateLinkTransforms();
  planning_scene->setCurrentState(*_robot->kinematic_state);
}

void aero::interface::AeroMotionPlanningInterface::processCollisionBox
(std::string _id, std::string _parent, aero::Transform _pose, aero::Vector3 _scale) {
  moveit_msgs::CollisionObject object;
  object.header.frame_id = _parent;
  object.id = _id;
  geometry_msgs::Pose p;
  p.position.x = _pose.translation().x();
  p.position.y = _pose.translation().y();
  p.position.z = _pose.translation().z();
  aero::Quaternion q(_pose.linear());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = _scale.x();
  primitive.dimensions[1] = _scale.y();
  primitive.dimensions[2] = _scale.z();
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(p);
  object.operation = object.ADD;
  planning_scene->processCollisionObjectMsg(object);
}

void aero::interface::AeroMotionPlanningInterface::processCollisionMesh
(std::string _id, std::string _parent, aero::Transform _pose, std::string _resource) {
  moveit_msgs::CollisionObject object;
  object.header.frame_id = _parent;
  object.id = _id;
  geometry_msgs::Pose p;
  p.position.x = _pose.translation().x();
  p.position.y = _pose.translation().y();
  p.position.z = _pose.translation().z();
  aero::Quaternion q(_pose.linear());
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  p.orientation.w = q.w();
  shapes::ShapeMsg mesh_msg;
  shapes::Mesh* mesh = shapes::createMeshFromResource(_resource);
  shapes::constructMsgFromShape(mesh, mesh_msg);
  object.meshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));
  object.mesh_poses.push_back(p);
  object.operation = object.ADD;
  planning_scene->processCollisionObjectMsg(object);
}

moveit_msgs::Constraints
aero::interface::AeroMotionPlanningInterface::constructGoalConstraintsFromIK
(aero::interface::AeroMoveitInterface::Ptr _robot, std::string _group_name, aero::Transform _goal, std::string _eef_link, double _tolerance1, double _tolerance2) {
  if (!_robot->setFromIK(_group_name, _goal, _eef_link)) {
    ROS_ERROR("End I.K. is not solvable!");
    return moveit_msgs::Constraints();
  } else {
    ROS_INFO("End I.K. is solvable.");
  }

  return
    kinematic_constraints::constructGoalConstraints(*_robot->kinematic_state, _robot->getJointModelGroup(_group_name), _tolerance1, _tolerance2);
}

bool aero::interface::AeroMotionPlanningInterface::solve
(planning_interface::MotionPlanRequest& _req, planning_interface::MotionPlanResponse& _res) {
  planning_interface::PlanningContextPtr context =
    planner_manager->getPlanningContext(planning_scene, _req, _res.error_code_);
  if (_res.error_code_.val == _res.error_code_.SUCCESS) {
    ROS_INFO("Context set successfully");
  } else {
    ROS_ERROR("Failed to set context");
    return false;
  }

  context->solve(_res);
  if (_res.error_code_.val != _res.error_code_.SUCCESS) {
    ROS_ERROR("Could not compute plan successfully");
    return false;
  }

  return true;
}

moveit_msgs::Constraints
aero::interface::AeroMotionPlanningInterface::constructPathConstraintsFromQuaternion
(std::string _eef_link, std::string _parent, aero::Quaternion _q) {
  geometry_msgs::QuaternionStamped qua;
  qua.header.frame_id = _parent;
  qua.quaternion.x = _q.x();
  qua.quaternion.y = _q.y();
  qua.quaternion.z = _q.z();
  qua.quaternion.w = _q.w();
  return kinematic_constraints::constructGoalConstraints(_eef_link, qua);
}

void aero::interface::AeroMotionPlanningInterface::displayTrajectory
(planning_interface::MotionPlanResponse _res) {
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit_msgs::MotionPlanResponse response;
  _res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher_.publish(display_trajectory);
}
