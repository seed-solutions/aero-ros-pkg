#include "aero_std/AeroMoveitInterface.hh"

aero::interface::AeroMoveitInterface::AeroMoveitInterface(ros::NodeHandle _nh, std::string _rd):
  AeroInterface(_nh),
  larm("larm"),larm_with_torso("larm_with_torso"),larm_with_lifter("larm_with_lifter"),
  rarm("rarm"),rarm_with_torso("rarm_with_torso"),rarm_with_lifter("rarm_with_lifter"),
  lifter("lifter"),upper_body("upper_body"),torso("torso"),head("head")
{
  // load robot model
  ROS_INFO("start loading robot model");
  robot_model_loader = robot_model_loader::RobotModelLoader(_rd);
  kinematic_model = robot_model_loader.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();// set all joints to 0.0

  ROS_INFO("start loading robot model for height only");
  robot_model_loader_ho = robot_model_loader::RobotModelLoader(_rd + "_height_only");
  kinematic_model_ho = robot_model_loader_ho.getModel();
  kinematic_state_ho = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_ho));
  kinematic_state_ho->setToDefaultValues();// set all joints to 0.0

  ROS_INFO("start loading robot model for on plane");
  robot_model_loader_op = robot_model_loader::RobotModelLoader(_rd + "_on_plane");
  kinematic_model_op = robot_model_loader_op.getModel();
  kinematic_state_op = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model_op));
  kinematic_state_op->setToDefaultValues();// set all joints to 0.0

  // JointModelGroup
  ROS_INFO("start loading joint model groups");
  jmg_larm = kinematic_model->getJointModelGroup("larm");
  jmg_larm_with_torso = kinematic_model->getJointModelGroup("larm_with_torso");
  jmg_larm_with_lifter = kinematic_model->getJointModelGroup("larm_with_lifter");
  jmg_larm_with_lifter_ho = kinematic_model_ho->getJointModelGroup("larm_with_lifter");
  jmg_larm_with_lifter_op = kinematic_model_op->getJointModelGroup("larm_with_lifter");
  jmg_rarm = kinematic_model->getJointModelGroup("rarm");
  jmg_rarm_with_torso = kinematic_model->getJointModelGroup("rarm_with_torso");
  jmg_rarm_with_lifter = kinematic_model->getJointModelGroup("rarm_with_lifter");
  jmg_rarm_with_lifter_ho = kinematic_model_ho->getJointModelGroup("rarm_with_lifter");
  jmg_rarm_with_lifter_op = kinematic_model_op->getJointModelGroup("rarm_with_lifter");
  jmg_lifter = kinematic_model->getJointModelGroup("lifter");


  display_publisher_ = _nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  angle_vector_publisher_ = _nh.advertise<trajectory_msgs::JointTrajectory>("/aero_controller/command", 1000);
  look_at_publisher_ = _nh.advertise<geometry_msgs::Point>("/look_at/target", 1000);
  planned_group_ = "";
  height_only_ = false;
  trajectory_ = std::vector<std::vector<double>>();
  trajectory_groups_ = std::vector<std::string>();
  joint_states_ = sensor_msgs::JointState();

  hand_grasp_client_ = _nh.serviceClient<aero_startup::AeroHandController>
    ("/aero_hand_controller");

  joint_states_subscriber_ = nh_.subscribe
    ("/joint_states",  1000, &aero::interface::AeroMoveitInterface::JointStateCallback, this);

  waist_service_ = nh_.serviceClient<aero_startup::AeroTorsoController>
    ("/aero_torso_controller");

  ROS_INFO("----------------------------------------");
  ROS_INFO("  AERO MOVEIT INTERFACE is initialized");
  ROS_INFO("----------------------------------------");
}

aero::interface::AeroMoveitInterface::~AeroMoveitInterface()
{
}

bool aero::interface::AeroMoveitInterface::plan(std::string _move_group){
  bool success = getMoveGroup(_move_group).plan(plan_);
  if (success) planned_group_ = _move_group;
  else planned_group_ = "";
  return success;
}

bool aero::interface::AeroMoveitInterface::execute(){
  if (planned_group_ == "") {
    ROS_WARN("execute error :: planned group not found");
    return false;
  }
  getMoveGroup(planned_group_).execute(plan_);
}

bool aero::interface::AeroMoveitInterface::move(std::string _move_group){
  bool success = plan(_move_group);
  if (!success) return false;
  viewTrajectory();
  success = execute();
  return success;
}

bool aero::interface::AeroMoveitInterface::solveIK(std::string _move_group, geometry_msgs::Pose _pose, std::string _eef_link){
  const robot_state::JointModelGroup* jmg_tmp;
  bool lifter_ik = false;

  if (_move_group == "larm") {
    jmg_tmp = jmg_larm;
  } else if (_move_group == "larm_with_torso") {
    jmg_tmp = jmg_larm_with_torso;
  } else if (_move_group == "larm_with_lifter") {
    lifter_ik = true;
    if (height_only_) jmg_tmp = jmg_larm_with_lifter_ho;
    else jmg_tmp = jmg_larm_with_lifter_op;
  } else if (_move_group == "rarm") {
    jmg_tmp = jmg_rarm;
  } else if (_move_group == "rarm_with_torso") {
    jmg_tmp = jmg_rarm_with_torso;
  } else if (_move_group == "rarm_with_lifter") {
    lifter_ik = true;
    if (height_only_) jmg_tmp = jmg_rarm_with_lifter_ho;
    else jmg_tmp = jmg_rarm_with_lifter_op;
  } else {
    ROS_WARN("IK error :: move_group [%s] doesn't exist", _move_group.c_str());
    return false;
  }

  if (lifter_ik) kinematic_state->enforceBounds(jmg_tmp);

  bool found_ik;
  if (_eef_link == "") found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, 10, 0.1);
  else found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, _eef_link, 10, 0.1);
  if (found_ik) getMoveGroup(_move_group).setJointValueTarget(*kinematic_state);
  if (found_ik || !lifter_ik) return found_ik;

  // if with lifter and first ik failed, trying another lifter's limit
  if (height_only_) switchOnPlane();
  else switchHeightOnly();

  if (_move_group == "larm_with_lifter") {
    if (height_only_) jmg_tmp = jmg_larm_with_lifter_ho;
    else jmg_tmp = jmg_larm_with_lifter_op;
  } else if (_move_group == "rarm_with_lifter") {
    if (height_only_) jmg_tmp = jmg_rarm_with_lifter_ho;
    else jmg_tmp = jmg_rarm_with_lifter_op;
  }

  kinematic_state->enforceBounds(jmg_tmp);
  if (_eef_link == "") found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, 10, 0.1);
  else found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, _eef_link, 10, 0.1);
  if (found_ik) getMoveGroup(_move_group).setJointValueTarget(*kinematic_state);
  return found_ik;
}

bool aero::interface::AeroMoveitInterface::solveIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, std::string _eef_link)
{
  return solveIK(aero::armAndRange2MoveGroup(_arm, _range), _pose, _eef_link);
}

bool aero::interface::AeroMoveitInterface::solveIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, aero::eef _eef)
{
  return solveIK(_arm, _range, _pose, armAndEEF2LinkName(_arm, _eef));
}

void aero::interface::AeroMoveitInterface::viewTrajectory(){
  if (planned_group_ == "") {
    ROS_WARN("view error :: planned group not found");
    return;
  }
  moveit_msgs::DisplayTrajectory display_trajectory;
  display_trajectory.trajectory_start = plan_.start_state_;
  display_trajectory.trajectory.push_back(plan_.trajectory_);
  display_publisher_.publish(display_trajectory);
}

void aero::interface::AeroMoveitInterface::setStartStateToCurrentState(std::string _move_group){
  getMoveGroup(_move_group).setStartStateToCurrentState();
}

moveit::planning_interface::MoveGroup &aero::interface::AeroMoveitInterface::getMoveGroup(std::string _move_group){
  if (_move_group == "larm") {
    return this->larm;
  } else if (_move_group == "larm_with_torso") {
    return this->larm_with_torso;
  } else if (_move_group == "larm_with_lifter") {
    return this->larm_with_lifter;
  } else if (_move_group == "rarm") {
    return this->rarm;
  } else if (_move_group == "rarm_with_torso") {
    return this->rarm_with_torso;
  } else if (_move_group == "rarm_with_lifter") {
    return this->rarm_with_lifter;
  } else if (_move_group == "upper_body") {
    return this->upper_body;
  } else if (_move_group == "head") {
    return this->head;
  } else if(_move_group == "torso") {
    return this->torso;
  } else if (_move_group == "lifter") {
    return this->lifter;
  } else {
    ROS_WARN("error :: move_group [%s] doesn't exist", _move_group.c_str());
    ros::shutdown();
  }
}

moveit::planning_interface::MoveGroup &aero::interface::AeroMoveitInterface::getMoveGroup(aero::arm _arm, aero::ikrange _range)
{
  std::string gname =  aero::armAndRange2MoveGroup(_arm, _range);

  return getMoveGroup(gname);
}

void aero::interface::AeroMoveitInterface::switchOnPlane()
{
  height_only_ = false;
}

void aero::interface::AeroMoveitInterface::switchHeightOnly()
{
  height_only_ = true;
}

void aero::interface::AeroMoveitInterface::setNamedTarget(std::string _move_group, std::string _target)
{
  getMoveGroup(_move_group).setNamedTarget(_target);
}

void aero::interface::AeroMoveitInterface::resetManipPose(int _time_ms)
{
  int time = 3000;
  if ( _time_ms != 0) time = _time_ms;
  setRobotStateToNamedTarget("upper_body", "reset-pose");
  sendAngleVectorAsync_("upper_body", time);
  usleep(time * 1000);
}

bool aero::interface::AeroMoveitInterface::moveWaist(double _x, double _z, int _time_ms)
{
  return moveWaist(static_cast<int>(_x * 1000), static_cast<int>(_z * 1000), _time_ms);
}

bool aero::interface::AeroMoveitInterface::moveWaist(int _x, int _z, int _time_ms)
{

  aero_startup::AeroTorsoController srv;
  srv.request.x = _x;
  srv.request.z = _z;
  if (_time_ms == 0) srv.request.coordinate = "world";
  else srv.request.coordinate = "world:" + std::to_string(_time_ms);
  if (!waist_service_.call(srv)) {
    ROS_ERROR("move waist failed service call");
    return false;
  }

  if (srv.response.status == "success") {
    setWaist(_x, _z);
    if (_time_ms == 0) usleep(static_cast<int>(srv.response.time_sec * 1000) * 1000);
    else usleep(_time_ms * 1000);
    return true;
  }
  return false;
}

bool aero::interface::AeroMoveitInterface::moveWaistLocal(double _x, double _z, int _time_ms)
{
  return moveWaistLocal(static_cast<int>(_x * 1000), static_cast<int>(_z * 1000), _time_ms);
}

bool aero::interface::AeroMoveitInterface::moveWaistLocal(int _x, int _z, int _time_ms)
{
  std::vector<double> pos = getWaistPositionRelative();
  return moveWaist(static_cast<int>(pos[0] * 1000) + _x, static_cast<int>(pos[1] * 1000) + _z, _time_ms);
}

bool aero::interface::AeroMoveitInterface::moveWaistAsync(double _x, double _z, int _time_ms)
{
  return moveWaistAsync(static_cast<int>(_x * 1000), static_cast<int>(_z * 1000), _time_ms);
}

bool aero::interface::AeroMoveitInterface::moveWaistAsync(int _x, int _z, int _time_ms)
{

  aero_startup::AeroTorsoController srv;
  srv.request.x = _x;
  srv.request.z = _z;
  if (_time_ms == 0) srv.request.coordinate = "world";
  else srv.request.coordinate = "world:" + std::to_string(_time_ms);

  if (!waist_service_.call(srv)) {
    ROS_ERROR("move waist failed service call");
    return false;
  }

  if (srv.response.status == "success") {
    setWaist(_x, _z);
    return true;
  }
  return false;
}

bool aero::interface::AeroMoveitInterface::moveWaistLocalAsync(double _x, double _z, int _time_ms)
{
  return moveWaistLocalAsync(static_cast<int>(_x * 1000), static_cast<int>(_z * 1000), _time_ms);
}

bool aero::interface::AeroMoveitInterface::moveWaistLocalAsync(int _x, int _z, int _time_ms)
{
  std::vector<double> pos = getWaistPositionRelative();
  return moveWaistAsync(static_cast<int>(pos[0] * 1000) + _x, static_cast<int>(pos[1] * 1000) + _z, _time_ms);
}

void aero::interface::AeroMoveitInterface::setWaist(double _x, double _z)
{
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(jmg_lifter, joint_values);

  joint_values[0] = _x;
  joint_values[1] = _z;
  kinematic_state->setJointGroupPositions(jmg_lifter, joint_values);
}
void aero::interface::AeroMoveitInterface::setWaist(int _x, int _z)
{
  setWaist(static_cast<double>(_x/1000.0), static_cast<double>(_z/1000.0));
}

Eigen::Vector3d aero::interface::AeroMoveitInterface::getWaistPosition()
{
  std::string link = "base_link";
  Eigen::Vector3d vec = kinematic_state->getGlobalLinkTransform(link).translation();
  return vec;
}

std::vector<double> aero::interface::AeroMoveitInterface::getWaistPositionRelative()
{
  std::vector<double> joint_values;// size = 2
  kinematic_state->copyJointGroupPositions(jmg_lifter, joint_values);
  return joint_values;
}

bool aero::interface::AeroMoveitInterface::solveIKSequence(aero::GraspRequest &_grasp)
{
  std::vector<double> result_mid(1);
  std::vector<double> av_ini;
  getRobotStateVariables(av_ini);

  std::string eef;


  if (_grasp.arm == aero::arm::rarm) eef = "r";
  else eef = "l";

  if (_grasp.eef == aero::eef::hand) eef = eef + "_hand_link";
  else if(_grasp.eef == aero::eef::grasp) eef = eef + "_eef_grasp_link";
  else if(_grasp.eef == aero::eef::pick) eef = eef + "_eef_pick_link";
  else if(_grasp.eef == aero::eef::index) eef = eef + "_index_tip_link";
  else if(_grasp.eef == aero::eef::thumb) eef = eef + "_thumb_tip_link";
  else eef = "";


  std::string res_m = solveIKOneSequence(_grasp.arm, _grasp.mid_pose, _grasp.mid_ik_range, av_ini, eef, result_mid);
  if (res_m == "") {
    ROS_INFO("mid ik failed");
    kinematic_state->setVariablePositions(av_ini);
    return false;
  }

  std::vector<double> result_end(1);
  std::vector<double> av_mid;
  getRobotStateVariables(av_mid);
  std::string res_e = solveIKOneSequence(_grasp.arm, _grasp.end_pose, _grasp.end_ik_range, av_mid, eef, result_end);

  if (res_e == "") {
    ROS_INFO("end ik failed");
    kinematic_state->setVariablePositions(av_ini);
    return false;
  }

  trajectory_.clear();
  trajectory_.reserve(2);
  trajectory_.push_back(result_mid);
  trajectory_.push_back(result_end);

  trajectory_groups_.clear();
  trajectory_groups_.reserve(2);
  trajectory_groups_.push_back(res_m);
  trajectory_groups_.push_back(res_e);

  kinematic_state->setVariablePositions(av_ini);
  return true;
}

std::string aero::interface::AeroMoveitInterface::solveIKOneSequence(aero::arm _arm, geometry_msgs::Pose _pose, aero::ikrange _ik_range, std::vector<double> _av_ini, std::string _eef_link, std::vector<double> &_result)
{
  bool status;
  std::string group = "larm";
  std::string gname = "";
  if (_arm == aero::arm::rarm) group = "rarm";

  // ik with arm
  std::cout << "ik" << std::endl;
  kinematic_state->setVariablePositions(_av_ini);
  gname = group;
  std::cout << "solve ik" << std::endl;
  status = solveIK(gname, _pose, _eef_link);
  std::cout << "solved ik" << std::endl;
  if (status && plan(gname)) {
    std::cout << "ccc" << std::endl;
    getRobotStateVariables(_result);
    return gname;
  }
  std::cout << "bbb" << std::endl;
  if (_ik_range == aero::ikrange::arm) return "";

  // ik with torso
  std::cout << "tor" << std::endl;
  kinematic_state->setVariablePositions(_av_ini);
  gname = group + "_with_torso";
  status = solveIK(gname, _pose, _eef_link);
  if (status && plan(gname)) {
    getRobotStateVariables(_result);
    return gname;
  }
  if (_ik_range == aero::ikrange::torso) return "";

  // ik with lifter
  std::cout << "lif" << std::endl;
  kinematic_state->setVariablePositions(_av_ini);
  gname = group + "_with_lifter";
  status = solveIK(group + "_with_lifter", _pose, _eef_link);
  if (status && plan(gname)) {
    getRobotStateVariables(_result);
    return gname;
  }  

  return "";
}

bool aero::interface::AeroMoveitInterface::moveSequence()
{
  // trajectory_に保存されたik結果列を順に実行する
  for (int i = 0; i < trajectory_.size(); ++i) {
    kinematic_state->setVariablePositions(trajectory_[i]);
    getMoveGroup(trajectory_groups_[i]).setJointValueTarget(*kinematic_state);
    sleep(1);
    move(trajectory_groups_[i]);
  }
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::openHand(bool _yes, aero::arm _arm)
{
  return openHand(_yes, _arm, -0.9, 0.8);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::openHand(bool _yes, aero::arm _arm,
                             float _warn, float _fail)
{
  aero_startup::AeroHandController srv;
  if (_arm == aero::arm::rarm)   srv.request.hand = "right";
  else srv.request.hand = "left";
  srv.request.thre_warn = _warn;
  srv.request.thre_fail = _fail;
  if (_yes) srv.request.command = "ungrasp";
  else srv.request.command = "grasp";
  
  if (!hand_grasp_client_.call(srv)) {
    ROS_ERROR("open/close hand failed service call");
    return false;
  }

  if (srv.response.status.find("success") != std::string::npos) {
    return true;
  }

  ROS_ERROR("%s", srv.response.status.c_str());
  return false;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::openHand(float _angle, aero::arm _arm)
{
  return openHand(_angle, _arm, -0.9, 0.8);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::openHand(float _angle, aero::arm _arm,
                             float _warn, float _fail)
{
  aero_startup::AeroHandController srv;
  if (_arm == aero::arm::rarm)   srv.request.hand = "right";
  else srv.request.hand = "left";
  srv.request.thre_warn = _warn;
  srv.request.thre_fail = _fail;
  srv.request.command = "grasp-angle";
  srv.request.larm_angle = _angle;
  srv.request.rarm_angle = _angle;

  if (!hand_grasp_client_.call(srv)) {
    ROS_ERROR("open/close hand failed service call");
    return false;
  }

  if (srv.response.status.find("success") != std::string::npos) {
    return true;
  }

  ROS_ERROR("%s", srv.response.status.c_str());
  return false;  
}

//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVector(aero::arm _arm, aero::ikrange _range, int _time_ms)
{
  sendAngleVectorAsync(_arm, _range, _time_ms);
  usleep(_time_ms * 1000);

  sleep(1); // guarantee action has finished
}

//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVector(int _time_ms, bool _move_waist)
{
  sendAngleVectorAsync(_time_ms, _move_waist);
  usleep(_time_ms * 1000);

  sleep(1); // guarantee action has finished
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVector(std::map<aero::joint, double> _av_map, int _time_ms, bool _move_waist)
{
  sendAngleVectorAsync(_av_map, _time_ms, _move_waist);
  usleep(_time_ms * 1000);

  sleep(1); // guarantee action has finished
}

//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVectorAsync(aero::arm _arm, aero::ikrange _range, int _time_ms)
{

  if (_range == aero::ikrange::lifter) {
    std::vector<double> joint_values = getWaistPositionRelative();
    if (!moveWaistAsync(joint_values[0], joint_values[1], _time_ms))
      {
        ROS_INFO("move waist failed");
        return;
      }
    sendAngleVectorAsync_( aero::armAndRange2MoveGroup(_arm, aero::ikrange::torso), _time_ms);
  } else {
    sendAngleVectorAsync_( aero::armAndRange2MoveGroup(_arm, _range), _time_ms);
  }
}

//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVectorAsync(int _time_ms, bool _move_waist)
{
  if (_move_waist) {
    std::vector<double> joint_values = getWaistPositionRelative();
    if(!moveWaistAsync(joint_values[0], joint_values[1], _time_ms))
      {
      ROS_INFO("move waist failed");
      return;
     }
  }
  sendAngleVectorAsync_("upper_body", _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync(std::map<aero::joint, double> _av_map, int _time_ms, bool _move_waist)
{
  setRobotStateVariables(_av_map);
  sendAngleVectorAsync(_time_ms, _move_waist);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt(double _x, double _y, double _z)
{
  geometry_msgs::Point msg;
  msg.x = _x;
  msg.y = _y;
  msg.z = _z;

  look_at_publisher_.publish(msg);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt(Eigen::Vector3d _target)
{
  setLookAt(_target.x(), _target.y(), _target.z());
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::resetLookAt()
{
  setLookAt(0.0, 0.0, 0.0);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(std::vector<double> &_av)
{
  kinematic_state->setVariablePositions(_av);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(std::map<std::string, double> &_map)
{
  kinematic_state->setVariablePositions(_map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(std::map<aero::joint, double> &_map)
{
  std::map<std::string, double> map;
  aero::jointMap2StringMap(_map, map);
  setRobotStateVariables(map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::vector<double> &_av)
{

  double* tmp;
  int num = static_cast<int>(kinematic_model->getVariableCount());
  tmp = kinematic_state->getVariablePositions();
  _av.clear();
  _av.reserve(num);
  _av.assign(tmp, tmp + num);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::map<std::string, double> &_map)
{
  _map.clear();

  for (auto it=aero::string_map.begin(); it != aero::string_map.end(); ++it) {
    _map[it->first] = kinematic_state->getVariablePosition(it->first);
  }
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::map<aero::joint, double> &_map)
{
  std::map<std::string, double> map_tmp;
  getRobotStateVariables(map_tmp);
  aero::stringMap2JointMap(map_tmp, _map);
}


//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToCurrentState()
{
  ros::spinOnce();
  std::map<std::string, double> map;
  for (auto it = aero::string_map.begin(); it != aero::string_map.end(); ++it) {
    auto itr = std::find(joint_states_.name.begin(), joint_states_.name.end(), it->first);
    if (itr == joint_states_.name.end()) continue;
    map[it->first] = joint_states_.position[static_cast<int>(itr - joint_states_.name.begin())];
  }
  kinematic_state->setVariablePositions(map);

  setHandsFromJointStates_();
  updateLinkTransforms();
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToNamedTarget(std::string _move_group, std::string _target)
{
  kinematic_state->setVariablePositions(getMoveGroup(_move_group).getNamedTargetValues(_target));
}
//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::setHand(aero::arm _arm, int _angle)
{
  std::string rl;
  if (_arm == aero::arm::rarm) rl = "r";
  else rl = "l";
  float rad = _angle * M_PI / 180.0;
  kinematic_state->setVariablePosition(rl + "_thumb_joint", rad);
  kinematic_state->setVariablePosition(rl + "_indexbase_joint", -rad);
}

//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::setHand(aero::arm _arm, double _radian)
{
  std::string rl;
  if (_arm == aero::arm::rarm) rl = "r";
  else rl = "l";
  kinematic_state->setVariablePosition(rl + "_thumb_joint", _radian);
  kinematic_state->setVariablePosition(rl + "_indexbase_joint", -_radian);
}

/////////////////////////////////////////////////
Eigen::Vector3d aero::interface::AeroMoveitInterface::getEEFPosition(aero::arm _arm, aero::eef _eef)
{
  std::string link = aero::armAndEEF2LinkName(_arm, _eef);
  Eigen::Vector3d vec = kinematic_state->getGlobalLinkTransform(link).translation();
  return vec;
}

/////////////////////////////////////////////////
Eigen::Quaterniond aero::interface::AeroMoveitInterface::getEEFOrientation(aero::arm _arm, aero::eef _eef)
{

  std::string link = aero::armAndEEF2LinkName(_arm, _eef);
  Eigen::Matrix3d mat = kinematic_state->getGlobalLinkTransform(link).rotation();
  Eigen::Quaterniond vec(mat);
  return vec;

}

/////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::updateLinkTransforms()
{
  kinematic_state->updateLinkTransforms();
}

/////////////////////////////////////////////////
Eigen::Affine3d aero::interface::AeroMoveitInterface::getCameraTransform()
{
  return kinematic_state->getGlobalLinkTransform("camera_link");
}

/////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVectorAsync_(std::vector<double> _av, std::vector<std::string> _joint_names, int _time_ms)
{
  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names.resize(_av.size());
  msg.joint_names = _joint_names;
  msg.points[0].positions.resize(_av.size());
  msg.points[0].positions = _av;
  msg.points[0].time_from_start = ros::Duration(_time_ms * 0.001);
  angle_vector_publisher_.publish(msg);

}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync_(std::string _move_group, int _time_ms)
{
  std::vector<double> av_mg;
  kinematic_state->copyJointGroupPositions(_move_group, av_mg);

  sendAngleVectorAsync_(av_mg, getMoveGroup(_move_group).getJointNames(), _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setHandsFromJointStates_()
{
  auto itr = std::find(joint_states_.name.begin(), joint_states_.name.end(), "r_thumb_joint");
  setHand(aero::arm::rarm, joint_states_.position[static_cast<int>(itr - joint_states_.name.begin())]);

  itr = std::find(joint_states_.name.begin(), joint_states_.name.end(), "l_thumb_joint");
  setHand(aero::arm::larm, joint_states_.position[static_cast<int>(itr - joint_states_.name.begin())]);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::JointStateCallback(const sensor_msgs::JointState::ConstPtr& _msg)
{
  joint_states_ = *_msg;
}
