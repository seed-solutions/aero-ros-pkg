#include "aero_addon/GraspAddon.hh"

using namespace aero;
using namespace addon;

GraspAddon::GraspAddon
(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _robot)
  : nh_(_nh)
{
  robot_ = _robot;

  tmb_loaded_ = false;
}

GraspAddon::~GraspAddon()
{
}

bool GraspAddon::readTumbleParameters(std::string _file)
{
  std::ifstream ifs(_file);
  if (ifs.fail())
    return false;

  std::string line;
  aero::parse::iparseLine(ifs, line); // skip first line (item id)
  tmb_reach_param_ = aero::parse::fparseLine(ifs, line);
  tmb_press_param_ = aero::parse::fparseLine(ifs, line);
  tmb_angle_param_ = aero::parse::fparseLine(ifs, line);

  aero::parse::fparseLine(ifs, line); // skip line
  aero::parse::fparseLine(ifs, line); // skip line
  tmb_omega_ = aero::parse::fparseLine(ifs, line) * M_PI / 180.0;
  ROS_INFO("omega %f", tmb_omega_);

  tmb_level_margin_ = aero::parse::fparseLine(ifs, line);

  aero::parse::iparseLine(ifs, line); // skip line (max_reach_iter)
  aero::parse::iparseLine(ifs, line); // skip line
  aero::parse::fparseLine(ifs, line); // skip line
  aero::parse::fparseLine(ifs, line); // skip line
  aero::parse::fparseLine(ifs, line); // skip line
  tmb_d_ = aero::parse::fparseLine(ifs, line);

  ifs.close();

  tmb_loaded_ = true;

  return true;
}

bool GraspAddon::solveBoxTumbleGrasp
(aero::arm _arm, Eigen::Vector3f _target, float _height, float _r)
{
  tmb_av_.clear();
  tmb_reach_av_.clear();
  tmb_lifter_av_.clear();

  target_ = _target.cast<double>();
  if (!tmb_loaded_) {
    std::string errmsg = "In tumble solve: parameter file was not loaded.";
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }

  if (!tmbInitiateBoxTumble(_arm, _target, tmb_d_, tmb_omega_))
    return false;

  if (!tmbAdjustInitialHandHeight(_arm, _target.z(), _height, tmb_level_margin_))
    return false;

  if (!tmbReachHandForward(_arm, _target.x(), tmb_reach_param_))
    return false;

  // save lifter position before tilt hand back
  std::map<aero::joint, double> pos0;
  robot_->getLifter(pos0);

  if (!tmbTiltHandBackward(tmb_angle_param_, _height, _r, tmb_press_param_))
    return false;

  // make sure hand can be raised for catching item
  std::map<aero::joint, double> pos1;
  robot_->getLifter(pos1);
  ROS_INFO("lifter final position %lf %lf", tmb_lifter_av_.back().first, tmb_lifter_av_.back().second);
  if (!robot_->setLifter(tmb_lifter_av_.back().first,
                         tmb_lifter_av_.back().second + tmb_press_param_ * 1.0, true)) {
    std::string errmsg = "In tumble solve: cannot catch item, I.K. failed.";
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }
  std::map<aero::joint, double> av0;
  robot_->getRobotStateVariables(av0);
  tmb_av_.push_back(av0);

  // lower hand for stable catch
  if (!robot_->setLifter( tmb_lifter_av_.back().first,
                          tmb_lifter_av_.back().second, true)) {
    std::string errmsg = "In tumble solve: cannot catch item, I.K. failed.";
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }
  std::map<aero::joint, double> av1;
  robot_->getRobotStateVariables(av1);
  tmb_av_.push_back(av1);

  return true;
}

bool GraspAddon::sendBoxTumbleGrasp(aero::arm _arm, int _power)
{
  if (tmb_av_.size() < 2 || tmb_reach_av_.size() == 0
      || tmb_lifter_av_.size() == 0)
    return false;

  // send hand height move
  ROS_INFO("send hand height move");
  robot_->setLookAt(target_);
  robot_->sendAngleVector(tmb_av_.at(1), 5000, aero::ikrange::lifter);

  // send lifter reach if required
  ROS_INFO("send lifter reach if required");
  int at = 2;
  if (tmb_av_.size() > 4) {
    robot_->setLookAt(target_);
    robot_->sendAngleVector(tmb_av_.at(at++), 3000, aero::ikrange::lifter);
  }
  // reach hand forward
  ROS_INFO("reach hand forward");
  robot_->setLookAt(target_);
  robot_->sendAngleVector(tmb_reach_av_.back(), 7000, aero::ikrange::lifter);


  // tilt hand backward
  ROS_INFO("tilt hand backward");
  robot_->setLookAt(target_);
  robot_->sendLifterTrajectory(tmb_lifter_av_, 1000 * tmb_lifter_av_.size());

  // grasping lifter move
  ROS_INFO("lifter1");
  robot_->setLookAt(target_);
  robot_->sendAngleVector(tmb_av_.at(at++), 2000, aero::ikrange::lifter);
  ROS_INFO("lifter2");
  robot_->setLookAt(target_);
  robot_->sendAngleVector(tmb_av_.at(at++), 2000, aero::ikrange::lifter);

  // grasp
  return robot_->sendGrasp(_arm, _power);
}

bool GraspAddon::tmbInitiateBoxTumble
(aero::arm _arm, Eigen::Vector3f _target, float _d, float _omega)
{
  ROS_INFO("omega %f in tmbInitiateBoxTumble", _omega);
  float default_base_height = 0.845;
  robot_->setRobotStateToNamedTarget("upper_body", "reset-pose");
  if (_arm == aero::arm::larm) {
    robot_->sendHand(_arm, _omega);
    robot_->setHand(_arm, _omega);
  } else {
    robot_->sendHand(_arm, -_omega);
    robot_->setHand(_arm, -_omega);
  }

  tmb_eef_.position.x = 0.6; // magic initial x position
  tmb_eef_.position.y = _target.y();
  tmb_eef_.position.z = _target.z();
  if (_arm == aero::arm::larm) tmb_eef_.orientation.x = 0.707107;
  else tmb_eef_.orientation.x = -0.707107;
  tmb_eef_.orientation.y = 0.0;
  tmb_eef_.orientation.z = 0.0;
  tmb_eef_.orientation.w = 0.707107;

  // find x position where inverse kinematics to y is solvable
  std::map<aero::joint, double> av_ini;
  robot_->getRobotStateVariables(av_ini);
  while ((!robot_->setFromIK(_arm, aero::ikrange::lifter, tmb_eef_,
                             aero::eef::grasp) ||
          !tmbSearchHandPose(_arm, _d))
         && tmb_eef_.position.x < 1.1) {
    tmb_eef_.position.x += 0.05;
    if (_arm == aero::arm::larm) tmb_eef_.orientation.x = 0.707107;
    else tmb_eef_.orientation.x = -0.707107;
    tmb_eef_.orientation.y = 0.0;
    tmb_eef_.orientation.z = 0.0;
    tmb_eef_.orientation.w = 0.707107;
    robot_->setRobotStateVariables(av_ini);
    printf("setting initial pose: %f\n", tmb_eef_.position.x);
  }

  if (tmb_eef_.position.x > 1.0) {
    std::string errmsg = "In tumble solve: could not solve initial pose.";
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }

  // add initial pose
  std::map<aero::joint, double> av;
  robot_->getRobotStateVariables(av);
  tmb_av_.push_back(av);

  return true;
}

bool GraspAddon::tmbAdjustInitialHandHeight
(aero::arm _arm, float _z, float _height, float _level_margin)
{
  float goal_height = _z + _height * 0.5 + _level_margin;
  auto index_pos = robot_->getEEFPosition(_arm, aero::eef::index);
  printf("setting hand height %lf -> %f\n", index_pos.z(), goal_height);
  std::map<aero::joint, double> pos;
  robot_->getLifter(pos);
  if (!robot_->setLifter(pos.at(aero::joint::lifter_x),
          pos.at(aero::joint::lifter_z) + goal_height - index_pos.z(), true)) {
    std::string errmsg = "In tumble solve: failed to send hand height.";
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }

  // add hand height move
  std::map<aero::joint, double> av;
  robot_->getRobotStateVariables(av);
  tmb_av_.push_back(av);

  return true;
}

bool GraspAddon::tmbSearchHandPose(aero::arm _arm, float _d)
{
  float dist = std::numeric_limits<float>::max();
  float index_x =
    robot_->getEEFPosition(_arm, aero::eef::index).x();
  float thumb_x =
    robot_->getEEFPosition(_arm, aero::eef::thumb).x();
  float pitch = 0.0;

  while (index_x > thumb_x) {
    if ((index_x - thumb_x) < dist)
      dist = index_x - thumb_x;

    if (dist < _d)
      return true;
    pitch += 0.01;

    Eigen::Quaternionf rot0;
    if (_arm == aero::arm::larm) rot0 = Eigen::Quaternionf(0.707107, 0.707107, 0.0, 0.0);
    else rot0 = Eigen::Quaternionf(0.707107, -0.707107, 0.0, 0.0);
    Eigen::Quaternionf rot =
      Eigen::Quaternionf(cos(-pitch), 0.0, sin(-pitch), 0.0) * rot0;
    tmb_eef_.orientation.x = rot.x();
    tmb_eef_.orientation.y = rot.y();
    tmb_eef_.orientation.z = rot.z();
    tmb_eef_.orientation.w = rot.w();
    if (!robot_->setFromIK(_arm, aero::ikrange::lifter, tmb_eef_,
                           aero::eef::grasp))
      return false;

    index_x = robot_->getEEFPosition(_arm, aero::eef::index).x();
    thumb_x = robot_->getEEFPosition(_arm, aero::eef::thumb).x();

    printf("setting hand pose: %lf, %lf %lf\n", index_x, thumb_x, index_x - thumb_x);
  }

  return false;
}

bool GraspAddon::tmbReachHandForward(aero::arm _arm, float _x, float _max_reach)
{
  auto index_pos = robot_->getEEFPosition(_arm, aero::eef::index);
  float to_object = _x - index_pos.x() + _max_reach;
  printf("to object distance: %f\n", to_object);

  // search hand forward
  float goal_given = tmb_eef_.position.x + to_object;
  float goal = std::min(goal_given, 0.8f); // for safe reach
  tmb_eef_.position.x += 0.01;
  printf("sending hand forward: %f\n", tmb_eef_.position.x);
  while (robot_->setFromIK(_arm, aero::ikrange::lifter, tmb_eef_,
                         aero::eef::grasp)
         && tmb_eef_.position.x < goal) {
    tmb_eef_.position.x += 0.01;
    std::map<aero::joint, double> av;
    robot_->getRobotStateVariables(av);
    tmb_reach_av_.push_back(av);
    printf("sending hand forward: %f\n", tmb_eef_.position.x);
  }
  if (tmb_eef_.position.x < goal)
    tmb_eef_.position.x -= 0.01;
  float remain = goal_given - tmb_eef_.position.x;
  printf("remaining: %f\n", remain);

  // // reset av to before reach
  // robot_->setRobotStateVariables(tmb_av_.at(tmb_av_.size() - 1));

  // when object is really far, try using lifter
  std::map<aero::joint, double> pos;
  robot_->getLifter(pos);
  if (remain > 0.01) {
    float tmp_goal = remain;
    while (tmp_goal > 0 &&
           !robot_->setLifter(pos.at(aero::joint::lifter_x) + tmp_goal,
                              pos.at(aero::joint::lifter_z), true))
      tmp_goal -= 0.005;
    remain -= tmp_goal;

    // add lifter move
    std::map<aero::joint, double> av;
    robot_->getRobotStateVariables(av);
    tmb_av_.push_back(av);
  }

  // abort if object is not reach-able even when using lifter
  if (remain > 0.01) {
    std::string errmsg = "In tumble solve: Object too far for stable grasp.";
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }

  return true;
}

bool GraspAddon::tmbTiltHandBackward
(float _theta, float _height, float _r, float _press_param)
{
  std::map<aero::joint, double> pos;
  robot_->getLifter(pos);

  float theta = 0.1;
  float x =
    pos.at(aero::joint::lifter_x) - _r + _height * sin(_theta) - _r * cos(_theta);
  float z = pos.at(aero::joint::lifter_z)
    - std::max(_height - _height * cos(_theta) - _r * sin(_theta) + _press_param,
               0.0);
  printf("tilting hand backward: %f, %f\n", x, z);

  while ((theta < _theta) && robot_->setLifter(x, z, true)) {
    theta += 0.05;

    x = pos.at(aero::joint::lifter_x)
      - (_r + _height * sin(theta) - _r * cos(theta));
    z = std::max(pos.at(aero::joint::lifter_z) -
                 (_height - _height * cos(theta) - _r * sin(theta) + _press_param),
                 static_cast<double>(z));
    printf("tilting hand backward: %f, %f\n", x, z);
    // add lifter movement
    std::map<aero::joint, double> av;
    robot_->getLifter(av);
    tmb_lifter_av_.push_back
      ({av.at(aero::joint::lifter_x), av.at(aero::joint::lifter_z)});
  }

  if (theta < _theta) {
    std::string errmsg = "In tumble solve: cannot tumble object, I.K. failed.";
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }

  return true;
}
