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

  scene_msg.name = "monitored_planning_scene";
  scene_msg.is_diff = true;

  display_publisher_ =
    _nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  scene_publisher_ =
    _nh.advertise<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", 1, true);

  contact_publisher_ =
    _nh.advertise<visualization_msgs::MarkerArray>("/moveit/collision/contacts", 1, true);
}

aero::interface::AeroMotionPlanningInterface::~AeroMotionPlanningInterface() {
}

void aero::interface::AeroMotionPlanningInterface::setCurrentState
(aero::interface::AeroMoveitInterface::Ptr _robot) {  
  _robot->updateLinkTransforms();
  planning_scene->setCurrentState(*_robot->kinematic_state);
  moveit::core::robotStateToRobotStateMsg(*_robot->kinematic_state, scene_msg.robot_state); 
}

aero::Transform aero::interface::AeroMotionPlanningInterface::getTransformInMoveGroupEEF
(aero::interface::AeroMoveitInterface::Ptr _robot, std::string _group_name, aero::arm _arm, aero::eef _eef, aero::Transform _mat) {
  std::map<aero::joint, double> av;
  _robot->getRobotStateVariables(av); // save current pose
  if (!_robot->setFromIK(_group_name, _mat, aero::eefLink(_arm, _eef))) {
    ROS_ERROR("failed to get transform for move group eef. input not valid!");
    return aero::Transform();
  }
  aero::Transform transform;
  _robot->getEndEffectorCoords(aero::arm::rarm, aero::eef::hand, transform);
  _robot->setRobotStateVariables(av); // set back to origin pose
  return transform;
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
  scene_msg.world.collision_objects.push_back(object);
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
  scene_msg.world.collision_objects.push_back(object);
}

// void aero::interface::AeroMotionPlanningInterface::processAttachedCollisionMesh
// (std::string _link_name, std::string _parent, aero::Transform _pose, std::string _resource, std::vector<std::string> _touch_links) {
//   moveit_msgs::AttachedCollisionObject at_object;
//   moveit_msgs::CollisionObject object;
//   object.header.frame_id = _parent;
//   object.id = _link_name;
//   geometry_msgs::Pose p;
//   p.position.x = _pose.translation().x();
//   p.position.y = _pose.translation().y();
//   p.position.z = _pose.translation().z();
//   aero::Quaternion q(_pose.linear());
//   p.orientation.x = q.x();
//   p.orientation.y = q.y();
//   p.orientation.z = q.z();
//   p.orientation.w = q.w();
//   shapes::ShapeMsg mesh_msg;
//   shapes::Mesh* mesh = shapes::createMeshFromResource(_resource);
//   shapes::constructMsgFromShape(mesh, mesh_msg);
//   object.meshes.push_back(boost::get<shape_msgs::Mesh>(mesh_msg));
//   object.mesh_poses.push_back(p);
//   object.operation = object.ADD;
//   at_object.link_name = _link_name;
//   at_object.object = object;
//   at_object.touch_links = _touch_links;
//   planning_scene->processAttachedCollisionObjectMsg(at_object);
// }

moveit_msgs::Constraints
aero::interface::AeroMotionPlanningInterface::constructGoalConstraints
(aero::interface::AeroMoveitInterface::Ptr _robot, std::string _group_name, double _tolerance1, double _tolerance2) {
  return
    kinematic_constraints::constructGoalConstraints(*_robot->kinematic_state, _robot->getJointModelGroup(_group_name), _tolerance1, _tolerance2);
}

moveit_msgs::Constraints
aero::interface::AeroMotionPlanningInterface::constructGoalConstraints
(std::string _origin, aero::Transform _goal, aero::arm _arm, double _tolerance1, double _tolerance2) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = _origin;
  pose.pose.position.x = _goal.translation().x();
  pose.pose.position.y = _goal.translation().y();
  pose.pose.position.z = _goal.translation().z();
  aero::Quaternion q(_goal.linear());
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();

  return
    kinematic_constraints::constructGoalConstraints(aero::eefLink(_arm, aero::eef::hand), pose, _tolerance1, _tolerance2);
}

bool aero::interface::AeroMotionPlanningInterface::plan
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

bool aero::interface::AeroMotionPlanningInterface::execute
(aero::interface::AeroMoveitInterface::Ptr _robot, planning_interface::MotionPlanResponse _res, int _duration, aero::ikrange _ikrange, bool _async) {
  moveit_msgs::MotionPlanResponse goal;
  _res.getMessage(goal);

  // always bypass robot interface (even if not efficient) so nothing is screwed up

  // get joint names
  std::vector<aero::joint> joint_names(goal.trajectory.joint_trajectory.joint_names.size());
  for (int i = 0; i < joint_names.size(); ++i)
    joint_names.at(i) = aero::str2joint(goal.trajectory.joint_trajectory.joint_names.at(i));

  // get time and angle joints
  std::vector<int> tm_seq(goal.trajectory.joint_trajectory.points.size());
  std::vector<std::map<aero::joint, double> > av_seq(goal.trajectory.joint_trajectory.points.size());
  for (int i = 0; i < av_seq.size(); ++i) {
    _robot->getRobotStateVariables(av_seq.at(i)); // make sure all joints are filled
    for (int j = 0; j < joint_names.size(); ++j)
      av_seq.at(i).at(joint_names.at(j)) =
        goal.trajectory.joint_trajectory.points.at(i).positions.at(j);
    tm_seq.at(i) = static_cast<int>(static_cast<float>(_duration) / av_seq.size());
  }

  // update model in scene
  moveit::core::RobotStatePtr rs_tmp(new moveit::core::RobotState(*_robot->kinematic_state));
  moveit::core::jointTrajPointToRobotState
    (goal.trajectory.joint_trajectory, static_cast<int>(av_seq.size()) - 1, *rs_tmp);
  moveit::core::robotStateToRobotStateMsg(*rs_tmp, scene_msg.robot_state);

  return _robot->sendTrajectory(av_seq, tm_seq, _ikrange, _async);
}

bool aero::interface::AeroMotionPlanningInterface::checkCollision
(aero::interface::AeroMoveitInterface::Ptr _robot, planning_interface::MotionPlanResponse _res) {
  collision_detection::CollisionRequest creq;
  collision_detection::CollisionResult cres;
  creq.contacts = true;
  creq.max_contacts = 100;
  creq.max_contacts_per_pair = 5;

  // find collision
  bool collision_found = false;
  moveit_msgs::MotionPlanResponse res;
  _res.getMessage(res);
  moveit::core::RobotStatePtr rs_tmp(new moveit::core::RobotState(*_robot->kinematic_state));
  for (int i = 0; i < res.trajectory.joint_trajectory.points.size(); ++i) {
    moveit::core::jointTrajPointToRobotState(res.trajectory.joint_trajectory, i, *rs_tmp);
    planning_scene->checkCollision(creq, cres, *rs_tmp);
    if (cres.collision) {
      ROS_WARN("collision at %d / %d", i, static_cast<int>(res.trajectory.joint_trajectory.points.size()));
      collision_found = true;
      break;
    }
  }

  // visualize collision
  if (collision_found && cres.contact_count > 0) {
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.g = 0.0;
    color.b = 1.0;
    color.a = 0.5;
    visualization_msgs::MarkerArray markers;
    collision_detection::getCollisionMarkersFromContacts
      (markers, "base_link", cres.contacts, color, ros::Duration(3.0), 0.05);
    contact_publisher_.publish(markers);
  }

  return collision_found;
}

moveit_msgs::Constraints
aero::interface::AeroMotionPlanningInterface::constructPathConstraints
(aero::arm _arm, std::string _parent, aero::Quaternion _q) {
  geometry_msgs::QuaternionStamped qua;
  qua.header.frame_id = _parent;
  qua.quaternion.x = _q.x();
  qua.quaternion.y = _q.y();
  qua.quaternion.z = _q.z();
  qua.quaternion.w = _q.w();
  return kinematic_constraints::constructGoalConstraints(aero::eefLink(_arm, aero::eef::hand), qua);
}

void aero::interface::AeroMotionPlanningInterface::displayScene() {
  scene_publisher_.publish(scene_msg);
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
