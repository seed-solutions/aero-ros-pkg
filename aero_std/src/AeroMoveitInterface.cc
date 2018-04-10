#include "aero_std/AeroMoveitInterface.hh"

//////////////////////////////////////////////////
aero::interface::AeroMoveitInterface::AeroMoveitInterface(ros::NodeHandle &_nh, const std::string &_rd) : aero::base_commander::AeroBaseCommander(_nh)
{
  ROS_INFO("start creating robot_interface");

  ri.reset(new aero::AeroRobotInterface(_nh));

  // service clients
#if USING_HAND
  hand_grasp_client_ = _nh.serviceClient<aero_startup::HandControl>
    ("/aero_hand_controller");
#endif

  // load robot model
  ROS_INFO("start loading robot model");
  robot_model_loader::RobotModelLoader rmlder(_rd);
  kinematic_model = rmlder.getModel();
  kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();// set all joints to 0.0

  // JointModelGroup
  ROS_INFO("start loading joint model groups");

#define _ADD_JMG_MAP(name)                      \
  { jmg_##name = kinematic_model->getJointModelGroup(#name); \
    if ( !jmg_##name ) {                                     \
      ROS_ERROR("JointModelGroup: " #name " is not found");  \
    } else {                                                 \
      joint_model_group_map[#name] = jmg_##name;             \
    }                                                        \
  }

  _ADD_JMG_MAP(larm);
  _ADD_JMG_MAP(larm_with_waist);
  _ADD_JMG_MAP(larm_with_lifter);
  _ADD_JMG_MAP(larm_with_torso);
  _ADD_JMG_MAP(rarm);
  _ADD_JMG_MAP(rarm_with_waist);
  _ADD_JMG_MAP(rarm_with_lifter);
  _ADD_JMG_MAP(rarm_with_torso);

  _ADD_JMG_MAP(lifter);
  _ADD_JMG_MAP(waist);
  _ADD_JMG_MAP(torso);
  _ADD_JMG_MAP(both_arms);
  _ADD_JMG_MAP(upper_body);
  _ADD_JMG_MAP(whole_body);
  _ADD_JMG_MAP(head);

  //variables
  tracking_mode_flag_ = false;

  send_trajectory_offset_ = 0.02;

  ROS_INFO("----------------------------------------");
  ROS_INFO("  AERO MOVEIT INTERFACE is initialized");
  ROS_INFO("----------------------------------------");

  //
  alc.reset(new aero::lookat_commander::AeroLookatCommander(_nh, this));
}

//////////////////////////////////////////////////
aero::interface::AeroMoveitInterface::~AeroMoveitInterface()
{
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(const std::vector<double> &_av)
{
  kinematic_state->setVariablePositions(_av);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(const std::map<std::string, double> &_map)
{
  kinematic_state->setVariablePositions(_map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateVariables(const aero::joint_angle_map &_map)
{
  std::map<std::string, double> map;
  aero::jointMap2StringMap(_map, map);
  setRobotStateVariables(map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setJoint(aero::joint _joint, double _angle)
{
  std::map<std::string, double> map;
  std::map<aero::joint, std::string>::const_iterator it = aero::joint_map.find(_joint);
  if (it != aero::joint_map.end()) {
    map[it->second] = _angle;
    kinematic_state->setVariablePositions(map);
  } else {
    ROS_WARN("can not find in joint_map");
  }
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToCurrentState()
{
  // TODO for hand ???
  robot_interface::joint_angle_map map;
  {
    boost::mutex::scoped_lock sl(ri_mutex_);
    ri->getActualPositions(map);
  }
  kinematic_state->setVariablePositions(map);

  updateLinkTransforms();
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToCurrentState(robot_state::RobotStatePtr &_robot_state)
{
  // TODO for hand ???
  robot_interface::joint_angle_map map;
  {
    boost::mutex::scoped_lock sl(ri_mutex_);
    ri->getActualPositions(map);
  }
  _robot_state->setVariablePositions(map);

  _robot_state->updateLinkTransforms();
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setRobotStateToNamedTarget(const std::string &_move_group, const std::string &_target)
{
  std::map<std::string, double> jnamemap;
  getJointModelGroup(_move_group)->getVariableDefaultPositions(_target, jnamemap);
  kinematic_state->setVariablePositions(jnamemap);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(const std::string &_move_group, const aero::Transform &_pose,
                                                     const std::string &_eef_link, int _attempts)
{
  ROS_DEBUG_STREAM("setFromIK " << _move_group << ", " << _eef_link << ", " << _pose);

  const robot_state::JointModelGroup* jmg_tmp = getJointModelGroup(_move_group);
  bool lifter_ik = false;

  if (jmg_tmp == NULL) {
    return false;
  }

  bool found_ik;
  if (_eef_link == "") {
    found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, _attempts, 0.1);
  } else {
    found_ik = kinematic_state->setFromIK(jmg_tmp, _pose, _eef_link, _attempts, 0.1);
  }
  //if (found_ik) getMoveGroup(_move_group).setJointValueTarget(*kinematic_state);
  return found_ik;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setFromIK(aero::arm _arm, aero::ikrange _range, const aero::Transform &_pose, aero::eef _eef, int _attempts)
{
  //return setFromIK(_arm, _range, _pose, armAndEEF2LinkName(_arm, _eef), _attempts);
  return setFromIK(aero::moveGroup(_arm, _range), _pose,
                   aero::eefLink(_arm, _eef), _attempts);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::setLifter(double _x, double _z, bool _check_lifter_ik)
{
  std::vector<double> ans_xz;

  return lifter_ik_(_x, _z, ans_xz);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::lifter_ik_(double _x, double _z, std::vector<double>& _ans_xz)
{
  ROS_DEBUG("lifter ik %f %f", _x, _z);
  if (!jmg_lifter) {
    ROS_ERROR("jointModelGroup lifter does not exists");
    return false;
  }
  aero::Transform initial_trans = aero::Translation(0, 0, 0.725) * aero::Quaternion::Identity(); // typeB_lifter
  aero::Transform diff_trans = aero::Translation(_x, 0, _z) * aero::Quaternion::Identity();
  aero::Transform base2top = initial_trans * diff_trans;

  const aero::Transform &base_trans = kinematic_state->getGlobalLinkTransform("lifter_base_link");
  aero::Transform _pose = base_trans * base2top;

  ROS_DEBUG_STREAM("pose: " << _pose);

  int _attempts = 3;
  bool found_ik = kinematic_state->setFromIK(jmg_lifter, _pose, _attempts, 0.1);

  ROS_DEBUG("lifter: found_ik %d", found_ik);

  _ans_xz.resize(2);
  _ans_xz[0] = _x;
  _ans_xz[0] = _z;

  return found_ik;
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt_(const aero::Vector3 &_pos,
                                                      robot_state::RobotStatePtr &_robot_state)
{
  auto neck = solveLookAt_(_pos, _robot_state);
  setNeck_(0.0, std::get<1>(neck), std::get<2>(neck), _robot_state);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt(const aero::Vector3 &_pos, bool _map_coordinate, bool _tracking, bool _record_topic)
{
  ROS_INFO("setTrackingMode is %d in setLookAt, looking for %f %f %f in %s",
           static_cast<int>(tracking_mode_flag_), _pos.x(), _pos.y(), _pos.z(),
           (_map_coordinate ? "map" : "base"));

  if (tracking_mode_flag_) {
    //geometry_msgs::Point msg;
    //msg.x = _x;
    //msg.y = _y;
    //msg.z = _z;
    if (_map_coordinate) {
      if (_tracking) {
        if (_record_topic) {
          previous_topic_ = "/look_at/target/map:"
            + std::to_string(_pos.x()) + "," + std::to_string(_pos.y()) + "," + std::to_string(_pos.z());
        }
        //look_at_publisher_map_.publish(msg);
        // look_at_mode
        alc->setTrackingMode(aero::tracking::map, _pos);
      } else {
        //look_at_publisher_map_static_.publish(msg);
        // look_at_mode
        alc->setTrackingMode(aero::tracking::map_static, _pos);
      }
    } else {
      if (_tracking) {
        if (_record_topic) {
          previous_topic_ = "/look_at/target:"
            + std::to_string(_pos.x()) + "," + std::to_string(_pos.y()) + "," + std::to_string(_pos.z());
        }
        //look_at_publisher_base_.publish(msg);
        // look_at_mode
        alc->setTrackingMode(aero::tracking::base, _pos);
      } else {
        //look_at_publisher_base_static_.publish(msg);
        // look_at_mode
        alc->setTrackingMode(aero::tracking::base_static, _pos);
      }
    }
  } else {
    setLookAt_(_pos, kinematic_state);
  }
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAt(double _x, double _y, double _z, bool _map_coordinate, bool _tracking, bool _record_topic)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Vector3 pos(_x, _y, _z);
  setLookAt(pos, _map_coordinate, _tracking, _record_topic);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAtTf(const std::string &_tf, bool _tracking)
{
  alc->setLookAtTf(_tf, !_tracking);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::resetLookAt()
{
  setNeck(0.0, 0.0, 0.0);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setNeck(double _r,double _p, double _y, bool _to_node)
{
  if (tracking_mode_flag_) {
    if (_to_node) {
      ROS_WARN("setNeck called in tracking mode! updating and sending model in node!");
      geometry_msgs::Point p;
      p.x = _r; p.y = _p; p.z = _y;
      //look_at_publisher_rpy_.publish(p);
      // look_at_mode
      alc->setNeckRPY(_r, _p, _y);
      return;
    } else {
      ROS_WARN("setNeck called in tracking mode! are you sure of what you are doing?");
    }
  }

  setNeck_(_r, _p, _y, kinematic_state);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setNeck_(double _r,double _p, double _y,
                                                    robot_state::RobotStatePtr &_robot_state)
{
  _robot_state->setVariablePosition("neck_r_joint", _r);
  _robot_state->setVariablePosition("neck_p_joint", _p);
  _robot_state->setVariablePosition("neck_y_joint", _y);

  _robot_state->enforceBounds( kinematic_model->getJointModelGroup("head"));
}

//////////////////////////////////////////////////
std::tuple<double, double, double> aero::interface::AeroMoveitInterface::solveLookAt(const aero::Vector3 &_pos)
{
  return solveLookAt_(_pos, kinematic_state);
}

//////////////////////////////////////////////////
std::tuple<double, double, double> aero::interface::AeroMoveitInterface::solveLookAt_(const aero::Vector3 &_pos,
                                                                                      robot_state::RobotStatePtr &_robot_state)
{
  double neck2eye = 0.2;
  double body2neck = 0.35;

  // get base position in robot coords
  _robot_state->updateLinkTransforms();

  std::string body_link = "body_link";
  aero::Vector3 base2body_p = _robot_state->getGlobalLinkTransform(body_link).translation();
  aero::Matrix3 base2body_mat = _robot_state->getGlobalLinkTransform(body_link).rotation();
  aero::Quaternion base2body_q(base2body_mat);

  aero::Vector3 pos_obj_rel = base2body_q.inverse() * (_pos - base2body_p) - aero::Vector3(0.0, 0.0, body2neck);

  double yaw = atan2(pos_obj_rel.y(), pos_obj_rel.x());
  double dis_obj = sqrt(pos_obj_rel.x() * pos_obj_rel.x()
                        + pos_obj_rel.y() * pos_obj_rel.y()
                        + pos_obj_rel.z() * pos_obj_rel.z());
  double theta = acos(neck2eye / dis_obj);
  double pitch_obj = atan2(- pos_obj_rel.z(), pos_obj_rel.x());
  double pitch = 1.5708 + pitch_obj - theta;

  return std::tuple<double, double, double>(0.0, pitch, yaw);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendNeckAsync_(int _time_ms,
                                                          robot_state::RobotStatePtr &_robot_state,
                                                          double _delay)
{
  const std::vector<std::string > jnames = {"neck_r_joint", "neck_p_joint", "neck_y_joint"};
  const std::vector<double > angles = {_robot_state->getVariablePosition("neck_r_joint"),
                                       _robot_state->getVariablePosition("neck_p_joint"),
                                       _robot_state->getVariablePosition("neck_y_joint")};
  {
    boost::mutex::scoped_lock sl(ri_mutex_);
    ros::Time start_time = ros::Time::now() + ros::Duration(_delay);
    //ri->sendAngles(jnames, angles, _time_ms * 0.001, start_time);
    // TODO: access robot dependant code
    ri->head->sendAngles(jnames, angles, _time_ms * 0.001, start_time); // send only head
  }
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendNeckAsync(int _time_ms)
{
  if (tracking_mode_flag_) {
    ROS_WARN("sendNeckAsync called in tracking mode! are you sure of what you are doing?");
  }
  sendNeckAsync_(_time_ms, kinematic_state, send_trajectory_offset_);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setLookAtTopic(std::string _topic, bool _record_topic)
{
  if (!tracking_mode_flag_) {
    ROS_WARN("must call setTrackingMode to true first for setLookAtTopic");
    return;
  }

  if (_record_topic) // usually false
    previous_topic_ = _topic;

  // std_msgs::String msg;
  // msg.data = _topic;

  if (_topic == "") {
    // msg.data = "/look_at/manager_disabled";
    // lookat_target_publisher_.publish(msg);
    //
    // lookat_topic_ = msg.data;
    lookat_topic_ = _topic;
    previous_topic_ = "/look_at/manager_disabled";
    alc->disableTrackingMode();
    return;
  } else if (_topic == "/look_at/manager_disabled") {
    ROS_WARN("note, /look_at/manager_disabled only valid from prev");
    ROS_WARN("please make sure to call setTrackingMode false");
    // aero_startup::AeroSendJoints srv;
    // if (!get_saved_neck_positions_.call(srv)) {
    //   ROS_WARN("failed to get saved neck positions.");
    // } else {
    //   // neck value set and send through node
    //   setNeck(srv.response.points.positions.at(0),
    //           srv.response.points.positions.at(1),
    //           srv.response.points.positions.at(2), true);
    // }
    // msg.data = "/look_at/manager_disabled";
    // lookat_target_publisher_.publish(msg);
    //
    // lookat_topic_ = msg.data;
    double r, p, y;
    alc->getNeckRPY(r, p, y);
    setNeck(r, p, y, true);
    lookat_topic_ = _topic;
    previous_topic_ = "/look_at/manager_disabled";
    return;
  } else if (_topic == "/look_at/previous") {
    ROS_INFO("got /look_at/previous of %s", previous_topic_.c_str());
    if (previous_topic_.find("/look_at/target") != std::string::npos) {
      auto pos = previous_topic_.find(":");
      std::string values = previous_topic_.substr(pos+1);
      auto posx = values.find(",");
      auto posy = values.find(",", posx+1);
      // geometry_msgs::Point msg;
      // msg.x = std::stof(values.substr(0, posx));
      // msg.y = std::stof(values.substr(posx + 1, posy - posx -1));
      // msg.z = std::stof(values.substr(posy + 1));
      aero::Vector3 targ(std::stof(values.substr(0, posx)),
                         std::stof(values.substr(posx + 1, posy - posx -1)),
                         std::stof(values.substr(posy + 1)));
      if (previous_topic_.find("map") != std::string::npos)
        // look_at_publisher_map_.publish(msg);
        alc->setTrackingMode(aero::tracking::map, targ);
      else
        // look_at_publisher_base_.publish(msg);
        alc->setTrackingMode(aero::tracking::base, targ);
    } else {
      setLookAtTopic(previous_topic_);
    }
    return;
  }

  //lookat_target_publisher_.publish(msg);
  // look_at_mode_
  lookat_topic_ = _topic;
  alc->setLookAtTopic(_topic); //// TODO
}

//////////////////////////////////////////////////
std::string aero::interface::AeroMoveitInterface::getLookAtTopic()
{
  //return lookat_topic_;
  return "";
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setTrackingMode(bool _yes)
{
  // std_srvs::SetBool req;
  // req.request.data = _yes;
  // if (activate_tracking_client_.call(req)) tracking_mode_flag_ = _yes;
  if (!_yes) {
    ROS_WARN("disabling tracking mode!");
    setLookAtTopic(""); // disable tracking
  } else {
    ROS_WARN("waitInterpolation disabled from setTrackingMode!");
  }
  tracking_mode_flag_ = _yes;
}

/////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::updateLinkTransforms()
{
  kinematic_state->updateLinkTransforms();
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::vector<double> &_av)
{
  double* tmp; // TODO: not compatible robot-interface angle-vector?
  int num = static_cast<int>(kinematic_model->getVariableCount());
  tmp = kinematic_state->getVariablePositions();
  _av.clear();
  _av.reserve(num);
  _av.assign(tmp, tmp + num);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::map<std::string, double> &_map)
{
  if (!jmg_whole_body) {
    ROS_ERROR("jointModelGroup whole_body does not exists");
    return;
  }
  _map.clear();
  const std::vector<std::string> &names = jmg_whole_body->getVariableNames();

  for (int i =0; i < names.size(); i++) {
    _map[names[i]] = kinematic_state->getVariablePosition(names[i]);
  }
#if 0 // ALL NAMES
  const std::vector<std::string> &names = kinematic_state->getVariableNames();
  double *pos = kinematic_state->getVariablePositions();
  for (int i =0; i < names.size(); i++) {
    _map[names[i]] = pos[i];
  }
#endif
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(aero::joint_angle_map &_map)
{
  std::map<std::string, double> map_tmp;
  getRobotStateVariables(map_tmp);
  aero::stringMap2JointMap(map_tmp, _map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(std::map<std::string, double> &_map, const std::string &_group)
{
  _map.clear();
  std::vector<double > angles;
  kinematic_state->copyJointGroupPositions(_group, angles);
  const std::vector<std::string > &names = getJointModelGroup(_group)->getVariableNames();
  for (int i =0; i < angles.size(); i++) {
    _map[names[i]] = angles[i];
  }
}
//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getRobotStateVariables(aero::joint_angle_map &_map, const std::string &_group)
{
  std::map<std::string, double> map_tmp;
  getRobotStateVariables(map_tmp, _group);
  aero::stringMap2JointMap(map_tmp, _map);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getCurrentState(aero::joint_angle_map &_map)
{
  robot_interface::joint_angle_map map;
  ri->getActualPositions(map);

  stringMap2JointMap(map, _map);
}

//////////////////////////////////////////////////
double aero::interface::AeroMoveitInterface::getJoint(aero::joint _joint)
{
  std::map<aero::joint, std::string>::const_iterator it = aero::joint_map.find(_joint);
  if (it != aero::joint_map.end()) {
    return kinematic_state->getVariablePosition(it->second);
  } else {
    ROS_WARN("can not find in joint_map");
  }
  return 0.0;
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setPoseVariables(const aero::pose &_pose)
{
  switch(_pose) {
  case aero::pose::reset:
    setRobotStateToNamedTarget("upper_body", "reset-manip-pose");
    break;
  case aero::pose::reset_manip:
    setRobotStateToNamedTarget("upper_body", "reset-pose");
    break;
  case aero::pose::move:
    setRobotStateToNamedTarget("upper_body", "move-safe");
    break;
  case aero::pose::initial:
    setRobotStateToNamedTarget("upper_body", "initial");
    break;
  default:
    break;
  }
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getPoseVariables(const aero::pose &_pose, aero::joint_angle_map &_map)
{
  aero::joint_angle_map save;
  getRobotStateVariables(save); // store variables
  setPoseVariables(_pose);
  getRobotStateVariables(_map); // setup return variable
  setRobotStateVariables(save); // restore variables
}

//////////////////////////////////////////////////
aero::Vector3 aero::interface::AeroMoveitInterface::getWaistPosition()
{
  updateLinkTransforms();
  std::string link = "waist_link";
  aero::Vector3 vec = kinematic_state->getGlobalLinkTransform(link).translation();
  return vec;
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getLifter(double &_x, double &_z)
{
  const aero::Transform &top_trans  = kinematic_state->getGlobalLinkTransform("lifter_top_link");
  const aero::Transform &base_trans = kinematic_state->getGlobalLinkTransform("lifter_base_link");

  aero::Transform diff_trans = base_trans.inverse() * top_trans;

  ROS_DEBUG_STREAM("getLifter: diff_trans: " << diff_trans);

  aero::Vector3 pos = diff_trans.translation();

  //_xz.resize(2);

  _x = pos.x();
  _z = 0.725 - pos.z(); // robot depend number, typeB_lifter
}

//////////////////////////////////////////////////
double aero::interface::AeroMoveitInterface::getHand(aero::arm _arm)
{
  std::string rl;
  if (_arm == aero::arm::rarm) rl = "r";
  else rl = "l";
  return kinematic_state->getVariablePosition(rl + "_thumb_joint");
}

/////////////////////////////////////////////////
aero::Vector3 aero::interface::AeroMoveitInterface::getEEFPosition(aero::arm _arm, aero::eef _eef)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords;
  getEndEffectorCoords(_arm, _eef, coords);
  aero::Vector3 pos(coords.translation());
  return pos;
}

/////////////////////////////////////////////////
aero::Quaternion aero::interface::AeroMoveitInterface::getEEFOrientation(aero::arm _arm, aero::eef _eef)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords;
  getEndEffectorCoords(_arm, _eef, coords);
  aero::Quaternion qq(coords.linear());
  return qq;
}

/////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::getEndEffectorCoords(aero::arm _arm, aero::eef _eef, aero::Transform &_coords)
{
  updateLinkTransforms();
  const std::string &link = aero::eefLink(_arm, _eef);
  _coords = kinematic_state->getGlobalLinkTransform(link);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorSync_(int _time_ms)
{
  ROS_DEBUG("sendAngleVectorSync_,wait_ %d", _time_ms);
  usleep(static_cast<int>(_time_ms * 0.8) * 1000);// wait 80 percent
  waitInterpolation_();
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendModelAngles(aero::arm _arm, aero::ikrange _range, int _time_ms, bool _async)
{
  sendAngleVectorAsync_( aero::moveGroup(_arm, _range), _time_ms);
  if (!_async) sendAngleVectorSync_(_time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendModelAngles(int _time_ms, aero::ikrange _move_waist, bool _async)
{
  sendAngleVectorAsync_(_time_ms, _move_waist);
  if (!_async) sendAngleVectorSync_(_time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVector(const aero::joint_angle_map &_map, int _time_ms)
{
  std::vector<double> av_mg;
  std::vector<std::string> j_names;
  for(auto it : _map) {
    av_mg.push_back(it.second);
    j_names.push_back(joint2str(it.first));
  }
  sendAngleVectorAsync_(av_mg, j_names, _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync_(int _time_ms, aero::ikrange _move_waist)
{
  std::vector<double> av_mg;
  std::vector<std::string> j_names;
  j_names = jmg_both_arms->getVariableNames();
  kinematic_state->copyJointGroupPositions("both_arms", av_mg);

  if (_move_waist == aero::ikrange::upperbody || _move_waist == aero::ikrange::wholebody) {
    std::vector<double> extra_av_mg;
    kinematic_state->copyJointGroupPositions("waist", extra_av_mg);
    const std::vector<std::string> &extra_j_names = jmg_waist->getVariableNames();

    std::copy(extra_av_mg.begin(),   extra_av_mg.end(),   std::back_inserter(av_mg));
    std::copy(extra_j_names.begin(), extra_j_names.end(), std::back_inserter(j_names));
  }
  if (_move_waist == aero::ikrange::arm_lifter || _move_waist == aero::ikrange::wholebody) {
    std::vector<double> extra_av_mg;
    kinematic_state->copyJointGroupPositions("lifter", extra_av_mg);
    const std::vector<std::string> &extra_j_names = jmg_lifter->getVariableNames();

    std::copy(extra_av_mg.begin(),   extra_av_mg.end(),   std::back_inserter(av_mg));
    std::copy(extra_j_names.begin(), extra_j_names.end(), std::back_inserter(j_names));
  }
  if (!tracking_mode_flag_) {
    std::vector<double> extra_av_mg;
    kinematic_state->copyJointGroupPositions("head", extra_av_mg);
    const std::vector<std::string> &extra_j_names = jmg_head->getVariableNames();

    std::copy(extra_av_mg.begin(),   extra_av_mg.end(),   std::back_inserter(av_mg));
    std::copy(extra_j_names.begin(), extra_j_names.end(), std::back_inserter(j_names));
  }
#if 0
  ROS_DEBUG("jn size: %ld, av size: %ld", j_names.size(), av_mg.size());
  for(int i = 0; i < j_names.size(); i++)
    ROS_DEBUG("%d %s %f", i, j_names[i].c_str(), av_mg[i]);
#endif
  sendAngleVectorAsync_(av_mg, j_names, _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync_(const std::string &_move_group, int _time_ms)
{
  std::vector<double> av_mg;
  kinematic_state->copyJointGroupPositions(_move_group, av_mg);
  std::vector<std::string> j_names;
  j_names = getJointModelGroup(_move_group)->getVariableNames();

  sendAngleVectorAsync_(av_mg, j_names, _time_ms);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::sendAngleVectorAsync_(const std::vector<double> &_av,
                                                                 const std::vector<std::string> &_joint_names, int _time_ms)
{
  boost::mutex::scoped_lock sl(ri_mutex_);
  ros::Time start_time = ros::Time::now() + ros::Duration(send_trajectory_offset_);
  // fill sent_command
  sent_command_.send_type  = aero::interface::send_type::angles;
  sent_command_.start_time = start_time;
  sent_command_.duration   = _time_ms*0.001;
  sent_command_.angle_vector = _av;
  sent_command_.joint_names  = _joint_names;
  //
  if (!tracking_mode_flag_) {
    ri->sendAngles(_joint_names, _av, _time_ms*0.001, start_time);
  } else {
    // while in tracking_mode, send angles other than head
    ri->sendAngles_wo_head(_joint_names, _av, _time_ms*0.001, start_time);
  }
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendTrajectory(const aero::trajectory &_trajectory, const std::vector<int> &_times,
                                                          aero::ikrange _move_lifter, bool _async)
{
  ROS_DEBUG("sendTrajectory %ld %ld", _trajectory.size(), _times.size());
  std::vector<robot_interface::angle_vector > avs;
  for(int i = 0; i < _trajectory.size(); i++) {
    robot_interface::joint_angle_map map;
    robot_interface::angle_vector av;
    aero::jointMap2StringMap(_trajectory[i], map);
    ri->convertToAngleVector(map, av);
    avs.push_back(av);
  }
  int total_tm = 0;
  robot_interface::time_vector tms;
  for(int i = 0; i < _times.size(); i++) {
    tms.push_back(_times[i]/1000.0);
    total_tm += _times[i];
  }
  std::vector<std::string > names;
  aero::controllerGroup(names, _move_lifter, !tracking_mode_flag_);
#if 0
  ROS_INFO("names %d", names.size());
  for (int i = 0; i < names.size(); i++) {
    ROS_INFO("  %d: %s", i, names[i].c_str());
  }
#endif
  {
    boost::mutex::scoped_lock sl(ri_mutex_);
    ros::Time start_time = ros::Time::now() + ros::Duration(send_trajectory_offset_);
    // fill sent_command
    sent_command_.send_type  = aero::interface::send_type::sequence;
    sent_command_.start_time = start_time;
    sent_command_.angle_vector_sequence = avs;
    sent_command_.controller_names = names;
    sent_command_.time_sequence = tms;
    //
    //ROS_INFO("angle_vector_sequence %d %d", avs.size(), tms.size());
    ri->send_angle_vector_sequence(avs, tms, names, start_time);
  }
  if(!_async) {
    waitInterpolation_(total_tm);
  }
  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendTrajectory(const aero::trajectory &_trajectory, int _time_ms,
                                                          aero::ikrange _move_lifter, bool _async)
{
  int sz = _trajectory.size();
  if (sz < 1) {
    return false;
  }
  std::vector<int> times(sz);
  for(int i = 0; i < sz; i++) {
    times.push_back(_time_ms/sz);
  }

  return sendTrajectory(_trajectory, times, _move_lifter, _async);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendLifter(double _x, double _z, int _time_ms, bool _local, bool _async)
{
  // TODO: _local
  std::map<std::string, double> _map;
  getRobotStateVariables(_map);
  ROS_DEBUG("sendLifter %f %f %d", _x, _z, _time_ms);
  if(setLifter(_x, _z)) {
    ROS_DEBUG("sendLifter: set success");
    std::vector<double> av;
    const std::vector<std::string> &names = jmg_lifter->getVariableNames();
    kinematic_state->copyJointGroupPositions("lifter", av);
    {
      boost::mutex::scoped_lock sl(ri_mutex_);
      ros::Time start_time = ros::Time::now() + ros::Duration(send_trajectory_offset_); // starting 0.05sec after now
      ROS_DEBUG("sendLifter: send");
      ri->sendAngles(names, av, _time_ms*0.001, start_time);
    }
    if(!_async) {
      ri->wait_interpolation("lifter");
      ROS_DEBUG("sendLifter: wait end");
    }
    setRobotStateVariables(_map); // revert state
    return true;
  }
  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::cancelLifter()
{
  // send cancel joints
  // why not use AeroSendJoints? -> to safe exit trajectory
  // but actually, cancel joints is not supported with AeroSendJoints

  // TODO: .....
  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::waitInterpolation(int _timeout_ms) {
  usleep(100 * 1000);// to avoid getting wrong controller state;
  return waitInterpolation_(_timeout_ms);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::waitInterpolation_(int _timeout_ms) {
  // TODO: thread safe wait interpolation
  if (tracking_mode_flag_) {
    return ri->wait_interpolation("without_head", _timeout_ms*1.1/1000.0 );
  } else {
    return ri->wait_interpolation( _timeout_ms*1.1/1000.0 );
  }
}

#if USING_HAND // hand
 //////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setHand(aero::arm _arm, double _radian)
{
  // robot dependant
  std::string rl;
  if (_arm == aero::arm::rarm) rl = "r";
  else rl = "l";
  kinematic_state->setVariablePosition(rl + "_thumb_joint", _radian);
  kinematic_state->setVariablePosition(rl + "_indexbase_joint", -_radian);
}

//////////////////////////////////////////////////
void aero::interface::AeroMoveitInterface::setHandsFromJointStates_()
{
#if 0 // TODO? or Not used anywhere
  auto itr = std::find(joint_states_.name.begin(), joint_states_.name.end(), "r_thumb_joint");
  setHand(aero::arm::rarm, joint_states_.position[static_cast<int>(itr - joint_states_.name.begin())]);

  itr = std::find(joint_states_.name.begin(), joint_states_.name.end(), "l_thumb_joint");
  setHand(aero::arm::larm, joint_states_.position[static_cast<int>(itr - joint_states_.name.begin())]);
#endif
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendGrasp(aero::arm _arm, int _power, float _tm_sec)
{
  aero_startup::HandControl srv;
  srv.request.command = aero_startup::HandControlRequest::COMMAND_GRASP;
  srv.request.power   = _power;
  srv.request.thre_warn = -0.9;
  srv.request.thre_fail = 0.2;
  srv.request.time_sec = _tm_sec;

  return callHandSrv_(_arm, srv);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendGraspFast(aero::arm _arm, int _power, float _thre_fail, float _tm_sec)
{
  aero_startup::HandControl srv;
  srv.request.command = aero_startup::HandControlRequest::COMMAND_GRASP_FAST;
  srv.request.power   = _power;
  srv.request.thre_fail = _thre_fail;
  srv.request.larm_angle = getHand(aero::arm::larm) * 180.0 / M_PI;
  srv.request.rarm_angle = getHand(aero::arm::rarm) * 180.0 / M_PI;
  srv.request.time_sec = _tm_sec;

  return callHandSrv_(_arm, srv);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::openHand(aero::arm _arm, float _tm_sec)
{
  aero_startup::HandControl srv;
  srv.request.command = aero_startup::HandControlRequest::COMMAND_UNGRASP;
  srv.request.power   = 0;
  srv.request.time_sec = _tm_sec;

  return callHandSrv_(_arm, srv);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendHand(aero::arm _arm, double _rad, float _tm_sec)
{

  if ( fabs(_rad) > 2.0) {
    ROS_WARN("openHand failed");
    ROS_WARN("specified angle is too large, please use double[rad]");
    return false;
  }

  aero_startup::HandControl srv;
  srv.request.command = aero_startup::HandControlRequest::COMMAND_GRASP_ANGLE;
  srv.request.power   = 0;
  srv.request.thre_warn = 0.0;
  srv.request.thre_fail = 0.0;
  srv.request.larm_angle = _rad * 180.0 / M_PI;
  srv.request.rarm_angle = _rad * 180.0 / M_PI;
  srv.request.time_sec = _tm_sec;

  return callHandSrv_(_arm, srv);
}

bool aero::interface::AeroMoveitInterface::callHandSrv_(const aero::arm &_arm, aero_startup::HandControl &_srv)
{
  if (_arm == aero::arm::rarm) _srv.request.hand = aero_startup::HandControlRequest::HAND_RIGHT;
  else _srv.request.hand = aero_startup::HandControlRequest::HAND_LEFT;

  if (!hand_grasp_client_.call(_srv)) {
    ROS_ERROR("open/close hand failed service call");
    return false;
  }

  if (_srv.response.success) return true;

  ROS_ERROR("HAND_SERVICE: %s", _srv.response.status.c_str());
  return false;
}
#endif // hand

/////////////////////////////////////////////////
const robot_state::JointModelGroup* aero::interface::AeroMoveitInterface::getJointModelGroup(const std::string &_move_group)
{
  auto it = joint_model_group_map.find(_move_group);
  if (it != joint_model_group_map.end()) {
    return it->second;
  }
  const robot_state::JointModelGroup* tmpj = kinematic_model->getJointModelGroup(_move_group);
  if (!tmpj) {
    joint_model_group_map[_move_group] = tmpj;
    return tmpj;
  }
  ROS_WARN("error :: joint_model_group [%s] doesn't exist", _move_group.c_str());
  return NULL;
}
/////////////////////////////////////////////////
const robot_state::JointModelGroup* aero::interface::AeroMoveitInterface::getJointModelGroup(aero::arm _arm, aero::ikrange _range)
{
  return getJointModelGroup(aero::moveGroup(_arm, _range));
}

//////////////////// TODO: rewrite grasp
#if USING_GRASP // grasp
//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::solveIKSequence(const aero::GraspRequest &_grasp)
{
  // save initial angles
  std::vector<double> av_ini;
  getRobotStateVariables(av_ini);

  std::string eef_link;
  eef_link = aero::eefLink(_grasp.arm, _grasp.eef);

  aero::joint_angle_map result_mid;
  std::string result_mid_range;
  aero::Transform graspMid = _grasp.mid_pose;

  if (solveIKOneSequence(_grasp.arm, graspMid, _grasp.mid_ik_range,
                         av_ini, _grasp.eef, result_mid_range, result_mid)) {
    ROS_INFO("mid ik failed");
    kinematic_state->setVariablePositions(av_ini);
    return false;
  }

  aero::joint_angle_map result_end;
  std::string result_end_range;
  aero::Transform graspEnd = _grasp.end_pose;

  std::vector<double> av_mid; // use av_mid as initial state for IK
  getRobotStateVariables(av_mid);
  if (solveIKOneSequence(_grasp.arm, graspEnd, _grasp.end_ik_range,
                         av_mid, _grasp.eef, result_end_range, result_end)) {
    ROS_INFO("end ik failed");
    kinematic_state->setVariablePositions(av_ini);
    return false;
  }

  trajectory_.clear();
  //trajectory_.reserve(2);
  trajectory_.push_back(result_mid);
  trajectory_.push_back(result_end);


  kinematic_state->setVariablePositions(av_ini);// return robot model to inital state
  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::solveIKOneSequence(
 aero::arm _arm, const aero::Transform &_pose, aero::ikrange _ik_range,
 const std::vector<double> &_av_initial, aero::eef _eef,
 std::string &_result_range, aero::joint_angle_map &_result)
{
  bool status;

  // ik with arm
  kinematic_state->setVariablePositions(_av_initial);
  status = setFromIK(_arm, aero::ikrange::arm, _pose, _eef);
  if (status) {
    getRobotStateVariables(_result);
    _result_range = aero::moveGroup(_arm, aero::ikrange::arm);
    return true;
  }
  if (_ik_range == aero::ikrange::arm) return false;

  // ik with waist
  kinematic_state->setVariablePositions(_av_initial);
  status = setFromIK(_arm, aero::ikrange::upperbody, _pose, _eef);
  if (status) {
    getRobotStateVariables(_result);
    _result_range = aero::moveGroup(_arm, aero::ikrange::upperbody);
    return true;
  }
  if (_ik_range == aero::ikrange::upperbody) return false;

  // ik with waist and lifter
  kinematic_state->setVariablePositions(_av_initial);
  status = setFromIK(_arm, aero::ikrange::wholebody, _pose, _eef);
  if (status) {
    getRobotStateVariables(_result);
    _result_range = aero::moveGroup(_arm, aero::ikrange::wholebody);
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendSequence(std::vector<int> _msecs) // sync
{
  // send angles saved in trajectory_ to real robot
  // information about arm and lifter is saved in trajectory_groups_

  if (trajectory_.empty()) {
    ROS_ERROR("no motion plan found");
    return false;
  }
  sendTrajectory(trajectory_, _msecs, aero::ikrange::wholebody, false);

  return true;
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendPickIK(const aero::GraspRequest &_grasp)
{
  // save initial angles
  aero::joint_angle_map av_ini;
  getRobotStateVariables(av_ini);

  aero::joint_angle_map av_mid_map, av_end_map;
  aero::Transform trans_mid = _grasp.mid_pose;
  aero::Transform trans_end = _grasp.end_pose;

  ROS_INFO("start solving IK");
  ROS_INFO_STREAM("end: " << trans_end);
  if (!setFromIK(_grasp.arm, _grasp.end_ik_range, trans_end, _grasp.eef)) {
    ROS_INFO("end ik failed");
    setRobotStateVariables(av_ini);
    return false;
  }
  setLookAt(trans_end.translation());
  getRobotStateVariables(av_end_map);

  ROS_INFO_STREAM("mid: " << trans_mid);
  if (!setFromIK(_grasp.arm, _grasp.mid_ik_range, trans_mid, _grasp.eef)) {
    ROS_INFO("mid ik failed");
    setRobotStateVariables(av_ini);
    return false;
  }
  setLookAt(trans_mid.translation());
  getRobotStateVariables(av_mid_map);

  ROS_INFO("start making trajectory");
  aero::trajectory trajectory;
  std::vector<int> times;
  int mid_time = 4000;
  int end_time = 4000;
  int num = 5;
  trajectory.reserve(num+1);
  times.reserve(num+1);
  trajectory.push_back(av_mid_map);
  times.push_back(mid_time);

  setRobotStateVariables(av_mid_map);

  int last_solved_num = -1;

  for (int i=0; i < num - 1; ++i) {
    aero::Transform tmp; // mid to end
    aero::mid_coords((i+1)/(double)num, trans_mid, trans_end, tmp);
    ROS_INFO_STREAM("tmp: " << i << ", trans: " << tmp);
    if (!setFromIK(_grasp.arm, _grasp.end_ik_range, tmp, _grasp.eef)) continue;
    ROS_INFO_STREAM("ik solved: " << i);
    setLookAt(tmp.translation());
    aero::joint_angle_map av_inner;
    getRobotStateVariables(av_inner);

    trajectory.push_back(av_inner);
    times.push_back((end_time / num) * (i - last_solved_num));
    last_solved_num = i;
  }

  trajectory.push_back(av_end_map);
  times.push_back((end_time / num) * (4 - last_solved_num));

  ROS_INFO("trajectory: %d relay points", static_cast<int>(times.size()) - 2);
  for(int i = 0; i < times.size(); i++) {
    ROS_INFO_STREAM("tm " << i << ": " <<  times[i]);
  }

  openHand(_grasp.arm);

  return sendTrajectory(trajectory, times, _grasp.end_ik_range);
}

//////////////////////////////////////////////////
bool aero::interface::AeroMoveitInterface::sendPlaceIK(const aero::GraspRequest &_grasp, double _push_height)
{
  return false;
#if 0
  // save initial angles
  aero::joint_angle_map av_ini;
  getRobotStateVariables(av_ini);

  aero::joint_angle_map av_mid,av_end,av_place;

  geometry_msgs::Pose mid_pose,end_pose,place_pose;
  mid_pose = _grasp.mid_pose;
  mid_pose.position.z += _push_height;
  end_pose = _grasp.end_pose;
  end_pose.position.z += _push_height;
  place_pose = _grasp.end_pose;


  ROS_INFO("solving IK");

  if (!setFromIK(_grasp.arm, _grasp.mid_ik_range, mid_pose, _grasp.eef)) {
    ROS_INFO("mid ik failed");
    setRobotStateVariables(av_ini);
    return false;
  }
  setLookAt(mid_pose.translate());
  getRobotStateVariables(av_mid);//save mid

  if (!setFromIK(_grasp.arm, _grasp.end_ik_range, end_pose, _grasp.eef)) {
    ROS_INFO("end ik failed");
    setRobotStateVariables(av_ini);
    return false;
  }
  setLookAt(end_pose.translate());
  getRobotStateVariables(av_end);//save end


  if (!setFromIK(_grasp.arm, _grasp.end_ik_range, place_pose, _grasp.eef)) {
    ROS_INFO("place ik failed");
    setRobotStateVariables(av_ini);
    return false;
  }
  getRobotStateVariables(av_place);//save end

  ROS_INFO("grasping IKs succeeded");

  ROS_INFO("making trajectory");
  aero::trajectory trajectory;
  std::vector<int> times;
  int mid_time = 4000;
  int end_time = 8000;
  int place_time = 2000;
  int num = 10;
  trajectory.reserve(num+1);
  times.reserve(num+1);
  trajectory.push_back(av_mid);
  times.push_back(mid_time);

  setRobotStateVariables(av_mid);
  geometry_msgs::Pose tmp, diff;
  tmp = mid_pose;
  diff.position.x = (end_pose.position.x - mid_pose.position.x) / num;
  diff.position.y = (end_pose.position.y - mid_pose.position.y) / num;
  diff.position.z = (end_pose.position.z - mid_pose.position.z) / num;
  Eigen::Quaterniond qua_mid{mid_pose.orientation.w,
      mid_pose.orientation.x ,mid_pose.orientation.y ,mid_pose.orientation.z};
  Eigen::Quaterniond qua_end{end_pose.orientation.w,
      end_pose.orientation.x ,end_pose.orientation.y ,end_pose.orientation.z};
  int last_solved_num = -1;
  ROS_INFO("mid pose %f %f %f, %f %f %f %f",
           mid_pose.position.x ,mid_pose.position.y ,mid_pose.position.z
           ,mid_pose.orientation.w,mid_pose.orientation.x ,mid_pose.orientation.y ,mid_pose.orientation.z);
  aero::ikrange range =  _grasp.end_ik_range;
  double lif_x_mid;
  double lif_z_mid;
  double lif_x_end;
  double lif_z_end;
  if (range == aero::ikrange::lifter) {
    range = aero::ikrange::wholebody;
    lif_x_mid = av_mid[aero::joint::lifter_x];
    lif_z_mid = av_end[aero::joint::lifter_z];
    lif_x_end = av_mid[aero::joint::lifter_x];
    lif_z_end = av_end[aero::joint::lifter_z];
  }
  for (int i=0; i < num - 1; ++i) {
    tmp.position.x += diff.position.x;
    tmp.position.y += diff.position.y;
    tmp.position.z += diff.position.z;
    Eigen::Quaterniond qua_path = qua_mid.slerp( static_cast<double>(i+1)/num, qua_end);
    tmp.orientation.w = qua_path.w();
    tmp.orientation.x = qua_path.x();
    tmp.orientation.y = qua_path.y();
    tmp.orientation.z = qua_path.z();

    ROS_INFO("%d th path pose %f %f %f, %f %f %f %f", i+1,
             tmp.position.x ,tmp.position.y ,tmp.position.z
             ,tmp.orientation.w,tmp.orientation.x ,tmp.orientation.y ,tmp.orientation.z);
    if (_grasp.end_ik_range == aero::ikrange::lifter) {
      double lif_x = lif_x_mid + (lif_x_end - lif_x_mid) * (i + 1);
      double lif_z = lif_z_mid + (lif_z_end - lif_z_mid) * (i + 1);
      setLifter(lif_x, lif_z);
    }
    continue;
    if (!setFromIK(_grasp.arm, range, tmp, _grasp.eef)) continue;
    aero::joint_angle_map av_inner;
    getRobotStateVariables(av_inner);
    trajectory.push_back(av_inner);
    times.push_back((end_time / num) * (i - last_solved_num));
    last_solved_num = i;
  }

  trajectory.push_back(av_end);
  times.push_back((end_time / num) * (4 - last_solved_num));
  ROS_INFO("end pose %f %f %f, %f %f %f %f",
           end_pose.position.x ,end_pose.position.y ,end_pose.position.z
           ,end_pose.orientation.w,end_pose.orientation.x ,end_pose.orientation.y ,end_pose.orientation.z);

  trajectory.push_back(av_place);
  times.push_back((place_time / num) * (4 - last_solved_num));
  ROS_INFO("place pose %f %f %f, %f %f %f %f",
           place_pose.position.x ,place_pose.position.y ,place_pose.position.z
           ,place_pose.orientation.w,place_pose.orientation.x ,place_pose.orientation.y ,place_pose.orientation.z);

  ROS_INFO("trajectory: %d relay points", static_cast<int>(times.size()) - 2);

  if (!sendTrajectory(trajectory, times, _grasp.end_ik_range)) {
    ROS_WARN("lifter ik failed, why???????");
    setRobotStateVariables(av_ini);
    return false;
  }

  openHand(_grasp.arm);
  sleep(1);

  aero::trajectory trajectory_return;
  std::vector<int> times_return;
  int trajectory_size = static_cast<int>(trajectory.size());
  int return_size = trajectory_size - 1;
  trajectory_return.reserve(return_size);
  times_return.reserve(return_size);
  for (int i = 0; i < return_size; ++i) {
    trajectory_return.push_back(trajectory[trajectory_size - 2 - i]);
    times_return.push_back(times[trajectory_size - 1 - i]);
  }

  if (!sendTrajectory(trajectory_return, times_return, _grasp.end_ik_range)) {
    ROS_WARN("lifter ik failed, why???????");
    setRobotStateVariables(av_ini);
    return false;
  }

  return true;
#endif
}
#endif // grasp

void aero::interface::AeroMoveitInterface::overwriteSpeed(float _speed_factor)
{
  ros::Time start_time = sent_command_.start_time;

  double remain_time;
  switch(sent_command_.send_type) {
  case aero::interface::send_type::angles:
    remain_time =
      ((start_time + ros::Duration(sent_command_.duration))
       - ros::Time::now()).toSec();
    break;
  case aero::interface::send_type::sequence:
    {
      double duration = 0;
      for(int i = 0; i < sent_command_.time_sequence.size(); i++) {
        duration += sent_command_.time_sequence[i];
      }
      remain_time =
        ((start_time + ros::Duration(duration)) - ros::Time::now()).toSec();
    }
    break;
  case aero::interface::send_type::stop_angles:
  case aero::interface::send_type::stop_sequence:
    remain_time = sent_command_.duration;
    break;
  default:
    ROS_WARN("overwriteSpeed was called, but no sent command");
    return;
    break;
  }

  if( remain_time <= 0 ) {
    ROS_WARN("overwriteSpeed was called, but previous trajectory finished");
    return;
  }

  if(_speed_factor == 0.0) {
    switch(sent_command_.send_type) {
    case aero::interface::send_type::stop_angles:
    case aero::interface::send_type::stop_sequence:
      // do nothing
      break;
    case aero::interface::send_type::angles:
      sent_command_.send_type = aero::interface::send_type::stop_angles;
      // TODO: stop motion
      break;
    case aero::interface::send_type::sequence:
      sent_command_.send_type = aero::interface::send_type::stop_sequence;
      // TODO: stop motion
      break;
    }
    return;
  }

  // setting for restart
  switch(sent_command_.send_type) {
  case aero::interface::send_type::stop_angles:
    sent_command_.send_type = aero::interface::send_type::angles;
    // TODO:
    break;
  case aero::interface::send_type::stop_sequence:
    sent_command_.send_type = aero::interface::send_type::sequence;
    // TODO:
    break;
  }

  // overwrite sent_command_
  switch(sent_command_.send_type) {
  case aero::interface::send_type::angles:
    {
      boost::mutex::scoped_lock sl(ri_mutex_);
      ros::Time start_time = ros::Time::now();
      sent_command_.duration = remain_time / _speed_factor;
      // sendAnglevectorAsync_
      if (!tracking_mode_flag_) {
        ri->sendAngles(sent_command_.joint_names,
                       sent_command_.angle_vector,
                       sent_command_.duration,
                       start_time);
      } else {
        // while in tracking_mode, send angles other than head
        ri->sendAngles_wo_head(sent_command_.joint_names,
                               sent_command_.angle_vector,
                               sent_command_.duration,
                               start_time);
      }
    }
    break;
  case aero::interface::send_type::sequence:
    {
      boost::mutex::scoped_lock sl(ri_mutex_);
      ros::Time time_now = ros::Time::now();
      double spent_time = (time_now - sent_command_.start_time).toSec();
      int start_idx = -1;
      double duration = 0;
      for(int i = 0; i < sent_command_.time_sequence.size(); i++) {
        duration += sent_command_.time_sequence[i];
        if(duration > spent_time) {
          start_idx = i;
          break;
        }
      }
      if(start_idx > 0) {
        std::vector<double > tms;
        std::vector<robot_interface::angle_vector > avs;
        for(int i = start_idx; i < sent_command_.time_sequence.size(); i++) {
          if (i == start_idx) {
            double tm = (duration - spent_time) / _speed_factor;
            if(tm > 0.05) { // add if remaining time is more than 0.05
              tms.push_back(tm);
              avs.push_back(sent_command_.angle_vector_sequence[i]);
            }
          } else {
            tms.push_back(sent_command_.time_sequence[i] / _speed_factor);
            avs.push_back(sent_command_.angle_vector_sequence[i]);
          }
        }
        if(avs.size() > 0) {
          sent_command_.start_time = time_now;
          sent_command_.angle_vector_sequence = avs;
          //sent_command_.controller_names = names;
          sent_command_.time_sequence = tms;
          ri->send_angle_vector_sequence(avs, tms, sent_command_.controller_names, time_now);
        }
      } else {
        //
      }
    }
    break;
  }
}
