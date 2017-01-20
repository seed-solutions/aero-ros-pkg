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
  planned_group_ = "";
  height_only_ = true;
  trajectory_ = std::vector<std::vector<double>>();
  trajectory_groups_ = std::vector<std::string>();

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

bool aero::interface::AeroMoveitInterface::solveIK(std::string _move_group, geometry_msgs::Pose _pose){
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

  bool found_ik;

  found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, 10, 0.1);  
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

bool aero::interface::AeroMoveitInterface::solveIKSequence(aero::GraspRequest &_grasp)
{
  std::vector<double> result_mid(1);
  std::vector<double> av_ini;
  getRobotStateVariables(av_ini);
  std::string res_m = solveIKOneSequence(_grasp.arm, _grasp.mid_pose, _grasp.mid_ik_range, av_ini, result_mid);
  if (res_m == "") return false;

  std::vector<double> result_end(1);
  std::vector<double> av_mid;
  getRobotStateVariables(av_mid);
  std::string res_e = solveIKOneSequence(_grasp.arm, _grasp.end_pose, _grasp.end_ik_range, av_mid, result_end);

  if (res_e == "") return false;

  trajectory_.clear();
  trajectory_.reserve(2);
  trajectory_.push_back(result_mid);
  trajectory_.push_back(result_end);

  trajectory_groups_.clear();
  trajectory_groups_.reserve(2);
  trajectory_groups_.push_back(res_m);
  trajectory_groups_.push_back(res_e);
  return true;
}

std::string aero::interface::AeroMoveitInterface::solveIKOneSequence(std::string _arm, geometry_msgs::Pose _pose, std::string _ik_range, std::vector<double> _av_ini, std::vector<double> &_result)
{
  bool status;
  std::string group = "larm";
  std::string gname = "";
  if (_arm == "right") group = "rarm";

  kinematic_state->setVariablePositions(_av_ini);
  status = solveIK(group, _pose);
  if (status) {
    getRobotStateVariables(_result);
    gname =group;
    return gname;
  }
  if (_ik_range == "arm") return gname;
  status = solveIK(group + "_with_torso", _pose);
  if (status) {
    getRobotStateVariables(_result);
    gname = group + "_with_torso";
    return gname;
  }
  if (_ik_range == "torso") return gname;

  status = solveIK(group + "_with_lifter", _pose);
  if (status) {
    getRobotStateVariables(_result);
    gname = group + "_with_lifter";
    return gname;
  }  

  if (height_only_) {
    switchOnPlane();
    if (group == "larm") kinematic_state->enforceBounds( jmg_larm_with_lifter_ho);
    else kinematic_state->enforceBounds( jmg_rarm_with_lifter_ho);
  } else {
    switchHeightOnly();
    if (group == "larm") kinematic_state->enforceBounds( jmg_larm_with_lifter_op);
    else kinematic_state->enforceBounds( jmg_rarm_with_lifter_op);
  }
  status = solveIK(group + "with_lifter", _pose);
  if (status) {
    getRobotStateVariables(_result);
    gname = group + "_with_lifter";
    return gname;
  }  

  return gname;
}

bool aero::interface::AeroMoveitInterface::moveSequence()
{
  // trajectory_に保存されたik結果列を順に実行する
  for (int i = 0; i < trajectory_.size(); ++i) {
    kinematic_state->setVariablePositions(trajectory_[i]);
    getMoveGroup(trajectory_groups_[i]).setJointValueTarget(*kinematic_state);
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
