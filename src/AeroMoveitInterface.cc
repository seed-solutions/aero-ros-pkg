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
  planned_group_ = "";
  height_only_ = true;
  trajectory_ = std::vector<std::vector<double>>();
  trajectory_groups_ = std::vector<std::string>();
  joint_states_ = sensor_msgs::JointState();

  hand_grasp_client_ = _nh.serviceClient<aero_startup::AeroHandController>
    ("/aero_hand_controller");

  joint_states_subscriber_ = nh_.subscribe
    ("/joint_states",  1000, &aero::interface::AeroMoveitInterface::JointStateCallback, this);

  ROS_INFO("AERO MOVEIT INTERFACE is initialized");
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
  std::cout << _move_group << std::endl;
  bool found_ik;
  if (_eef_link == "") found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, 10, 0.1);
  else found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, _eef_link, 10, 0.1);
  if (found_ik) getMoveGroup(_move_group).setJointValueTarget(*kinematic_state);

  return found_ik;
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
  // named-targetを目標値にセットします
  // named_targetはaero_moveit_config/config/AeroUpperRobot.srdfに記載されているgroup-stateです
  // ロボットの初期姿勢は("upper_body", "reset-pose")です
  getMoveGroup(_move_group).setNamedTarget(_target);
}

void aero::interface::AeroMoveitInterface::moveWaist(double _x, double _z)
{
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(jmg_lifter, joint_values);

  joint_values[0] = _x;
  joint_values[1] = _z;
  kinematic_state->setJointGroupPositions(jmg_lifter, joint_values);
  lifter.setStartStateToCurrentState();
  lifter.setJointValueTarget(*kinematic_state);
  bool success = plan("lifter");
  if (!success) return;
  success = execute();
}

void aero::interface::AeroMoveitInterface::moveWaistLocal(double _x, double _z)
{
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(jmg_lifter, joint_values);

  joint_values[0] = joint_values[0] + _x;
  joint_values[1] = joint_values[1] + _z;
  kinematic_state->setJointGroupPositions(jmg_lifter, joint_values);
  lifter.setStartStateToCurrentState();
  lifter.setJointValueTarget(*kinematic_state);
  bool success = plan("lifter");
  if (!success) return;
  success = execute();
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

  if (_ik_range == aero::ikrange::on_plane) { // ik on plane only
    kinematic_state->setVariablePositions(_av_ini);
    gname = group + "_with_lifter";
    switchOnPlane();
    if (group == "larm") kinematic_state->enforceBounds( jmg_larm_with_lifter_op);
    else kinematic_state->enforceBounds( jmg_rarm_with_lifter_op);
    status = solveIK(gname, _pose, _eef_link);
    if (status && plan(gname)) {
      getRobotStateVariables(_result);
      return gname;
    }
    return "";
  }

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
  if (height_only_) {
    if (group == "larm") kinematic_state->enforceBounds( jmg_larm_with_lifter_ho);
    else kinematic_state->enforceBounds( jmg_rarm_with_lifter_ho);
  } else {
    if (group == "larm") kinematic_state->enforceBounds( jmg_larm_with_lifter_op);
    else kinematic_state->enforceBounds( jmg_rarm_with_lifter_op);
  }
  status = solveIK(group + "_with_lifter", _pose, _eef_link);
  if (status && plan(gname)) {
    getRobotStateVariables(_result);
    return gname;
  }  


  kinematic_state->setVariablePositions(_av_ini);
  gname = group + "_with_lifter";
  if (height_only_) {
    switchOnPlane();
    if (group == "larm") kinematic_state->enforceBounds( jmg_larm_with_lifter_op);
    else kinematic_state->enforceBounds( jmg_rarm_with_lifter_op);
  } else {
    switchHeightOnly();
    if (group == "larm") kinematic_state->enforceBounds( jmg_larm_with_lifter_ho);
    else kinematic_state->enforceBounds( jmg_rarm_with_lifter_ho);
  }
  status = solveIK(gname, _pose, _eef_link);
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


void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::vector<double> &_av)
{
  // kinematic_stateが持っているすべての関節値を引数の_avに保存します。
  // solveIKを解くとkinematic_stateの角度列が変更されるので、それを取り出すのなどに使えます。
  // kinematic_stateに代入する場合は、this.kinematic_state->setVariablePositions(_av);を実行してください。

  double* tmp;
  // Aeroには無いが多変数関節などもありうるので、関節数ではなく関節変数の数を取得
  int num = static_cast<int>(kinematic_model->getVariableCount());
  // double* が返ってくる
  tmp = kinematic_state->getVariablePositions();
  // std::vectorに入れる
  _av.clear();
  _av.reserve(num);
  _av.assign(tmp, tmp + num);
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

void aero::interface::AeroMoveitInterface::sendAngleVector(std::string _move_group, std::vector<double> _av, int _time_ms)
{
  std::vector<double> av_original;
  getRobotStateVariables(av_original);// save
  std::vector<double> av_mg;
  kinematic_state->copyJointGroupPositions(_move_group, av_mg);
  setRobotStateVariables(av_original);// return

  sendAngleVector_(av_mg, getMoveGroup(_move_group).getJointNames(), _time_ms);
}

//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVector(std::string _move_group, int _time_ms)
{
  std::vector<double> av;
  getRobotStateVariables(av);
  sendAngleVector(_move_group, av, _time_ms);
}

//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVector(std::vector<double> _av, int _time_ms)
{
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(jmg_lifter, joint_values);

  Eigen::Vector3f waist;
  waist.x() = joint_values[0] * 1000;
  waist.z() = joint_values[1] * 1000;
  if(!MoveWaist(waist, "world")) ROS_INFO("move waist failed");
  sendAngleVector("upper_body", _av, _time_ms);
}
//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVector(int _time_ms)
{
  std::vector<double> av;
  getRobotStateVariables(av);
  sendAngleVector(av, _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(std::vector<double> &_av)
{
  kinematic_state->setVariablePositions(_av);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToCurrentState()//doubtful
{
  ros::spinOnce();
  kinematic_state->setVariablePositions(joint_states_.name, joint_states_.position);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToNamedTarget(std::string _move_group, std::string _target)
{
  kinematic_state->setVariablePositions(getMoveGroup(_move_group).getNamedTargetValues(_target));
}
//////////////////////////////////////////////////

void aero::interface::AeroMoveitInterface::sendAngleVector_(std::vector<double> _av, std::vector<std::string> _joint_names, int _time_ms)
{
  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names.resize(_av.size());
  msg.joint_names = _joint_names;
  msg.points[0].positions.resize(_av.size());
  msg.points[0].positions = _av;
  msg.points[0].time_from_start = ros::Duration(_time_ms * 0.001);
  usleep(1000 * 1000);
  angle_vector_publisher_.publish(msg);

}

void aero::interface::AeroMoveitInterface::JointStateCallback(const sensor_msgs::JointState::ConstPtr& _msg)
{
  joint_states_ = *_msg;
}
