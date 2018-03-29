#include <aero_std/AeroMoveitInterface.hh>
////// for backward compatibility

void aero::interface::AeroMoveitInterfaceDeprecated::sendAngleVector(aero::arm _arm, aero::ikrange _range, int _time_ms, bool _async)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendModelAngles(_arm, _range, _time_ms, _async);
}
void aero::interface::AeroMoveitInterfaceDeprecated::sendAngleVector(int _time_ms, aero::ikrange _move_waist, bool _async)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendModelAngles(_time_ms, _move_waist, _async);
}
void aero::interface::AeroMoveitInterfaceDeprecated::getLifter(aero::joint_angle_map &_xz)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  double x, z;
  getLifter(x, z);
  _xz[aero::joint::lifter_x] = x;
  _xz[aero::joint::lifter_z] = z;
}
//
bool aero::interface::AeroMoveitInterfaceDeprecated::setFromIK(std::string _move_group, geometry_msgs::Pose _pose, std::string _eef_link, int _attempts)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform pose;
  tf::poseMsgToEigen(_pose, pose);
  setFromIK(_move_group, pose, _eef_link, _attempts);
}
bool aero::interface::AeroMoveitInterfaceDeprecated::setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, std::string _eef_link, int _attempts)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform pose;
  tf::poseMsgToEigen(_pose, pose);
  setFromIK(moveGroup(_arm, _range), pose, _eef_link, _attempts);
}
bool aero::interface::AeroMoveitInterfaceDeprecated::setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, aero::eef _eef, int _attempts)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform pose;
  tf::poseMsgToEigen(_pose, pose);
  setFromIK(_arm, _range, pose, _eef, _attempts);
}
//
bool aero::interface::AeroMoveitInterfaceDeprecated::setFromIK(std::string _move_group, Vector3 _pos, Quaternion _qua, std::string _eef_link, int _attempts)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform pose = aero::Translation(_pos) * _qua;
  setFromIK(_move_group, pose, _eef_link, _attempts);
}
bool aero::interface::AeroMoveitInterfaceDeprecated::setFromIK(aero::arm _arm, aero::ikrange _range, Vector3 _pos, Quaternion _qua, std::string _eef_link, int _attempts)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform pose = aero::Translation(_pos) * _qua;
  setFromIK(moveGroup(_arm, _range), pose, _eef_link, _attempts);
}
bool aero::interface::AeroMoveitInterfaceDeprecated::setFromIK(aero::arm _arm, aero::ikrange _range, Vector3 _pos, Quaternion _qua, aero::eef _eef, int _attempts)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform pose = aero::Translation(_pos) * _qua;
  setFromIK(_arm, _range, pose, _eef, _attempts);
}
//
void aero::interface::AeroMoveitInterfaceDeprecated::getResetManipPose(aero::joint_angle_map &_map)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::joint_angle_map save;
  getRobotStateVariables(save); // store variables
  setPoseVariables(aero::pose::reset_manip);
  getRobotStateVariables(_map); // setup return variable
  setRobotStateVariables(save); // restore variables
}
void aero::interface::AeroMoveitInterfaceDeprecated::sendResetManipPose(int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::joint_angle_map save;
  getRobotStateVariables(save); // store variables
  setPoseVariables(aero::pose::reset_manip);
  sendModelAngles(_time_ms);
  setRobotStateVariables(save); // restore variables
  waitInterpolation();
}
//
void aero::interface::AeroMoveitInterfaceDeprecated::sendAngleVectorAsync(aero::arm _arm, aero::ikrange _range, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendModelAngles(_arm, _range, _time_ms, true);
}
void aero::interface::AeroMoveitInterfaceDeprecated::sendAngleVectorAsync(int _time_ms, aero::ikrange _move_waist)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendModelAngles(_time_ms, _move_waist, true);
}
void aero::interface::AeroMoveitInterfaceDeprecated::sendAngleVectorAsync(aero::joint_angle_map _av_map, int _time_ms, aero::ikrange _move_waist)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendAngleVector(_av_map, _time_ms);
}
// sendAngleVectorSequence
bool aero::interface::AeroMoveitInterfaceDeprecated::sendTrajectoryAsync(aero::trajectory _trajectory, std::vector<int> _times, aero::ikrange _move_lifter)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendTrajectory(_trajectory, _times, _move_lifter, true);
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendTrajectoryAsync(aero::trajectory _trajectory, int _time_ms, aero::ikrange _move_lifter)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendTrajectory(_trajectory, _time_ms, _move_lifter, true);
}
//
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifter(double _x, double _z, int _time_ms, bool _local, bool _async)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifter(_x, _z, _time_ms, false, false); // local=false, async=false
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifter(int _x, int _z, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifter(_x*0.001, _z*0.001, _time_ms, false, false); // local=false, async=false
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterLocal(double _x, double _z, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifter(_x, _z, _time_ms, true, false); // local=true, async=false
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterLocal(int _x, int _z, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifter(_x*0.001, _z*0.001, _time_ms, true, false); // local=true, async=false
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterAsync(double _x, double _z, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifter(_x, _z, _time_ms, false, true); // local=false, async=true
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterAsync(int _x, int _z, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifter(_x*0.001, _z*0.001, _time_ms, false, true); // local=false, async=true
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterLocalAsync(double _x, double _z, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifter(_x, _z, _time_ms, true, true); // local=true, async=true
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterLocalAsync(int _x, int _z, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifter(_x*0.001, _z*0.001, _time_ms, true, true); // local=true, async=true
}
//
bool aero::interface::AeroMoveitInterfaceDeprecated::cancelLifter()
{
  ROS_FATAL_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  // cancelMotion
}
//???
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifterTrajectoryAsync(_trajectory, _times);
  waitInterpolation();
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  sendLifterTrajectoryAsync(_trajectory, _time_ms);
  waitInterpolation();
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  std::vector<double > av_initial;
  getRobotStateVariables(av_initial); // store state
  aero::trajectory traj;
  for(auto p : _trajectory) {
    aero::joint_angle_map map;
    setLifter(p.first, p.second);
    getRobotStateVariables(map, "lifter");
    traj.push_back(map);
  }
  setRobotStateVariables(av_initial); // restore state
  sendTrajectory(traj, _times);
}
bool aero::interface::AeroMoveitInterfaceDeprecated::sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, int _time_ms)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  std::vector<int> tms;
  int sz = _trajectory.size();
  for(int i = 0; i < sz; i++) {
    tms.push_back(_time_ms/sz);
  }
  sendLifterTrajectoryAsync(_trajectory, tms);
}

//// BASE
geometry_msgs::Pose aero::interface::AeroMoveitInterfaceDeprecated::getCurrentPose(std::string _map)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords;
  bool ret = getCurrentCoords(coords, _map);
  geometry_msgs::Pose p;
  tf::poseEigenToMsg(coords, p);
  return p;
}

geometry_msgs::Pose aero::interface::AeroMoveitInterfaceDeprecated::getLocationPose(std::string _location)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords;
  bool ret = getLocationCoords(coords, _location);
  geometry_msgs::Pose p;
  tf::poseEigenToMsg(coords, p);
  return p;
}

void aero::interface::AeroMoveitInterfaceDeprecated::goPosAsync(double _x, double _y, double _rad)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  goPos(_x, _y, _rad, 20000,//default
        true);
}

void aero::interface::AeroMoveitInterfaceDeprecated::moveToAsync(std::string _location)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  moveTo(_location, true);
}

void aero::interface::AeroMoveitInterfaceDeprecated::moveToAsync(Vector3 _point)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords = aero::Translation(_point) * aero::Quaternion::Identity();
  moveTo(coords, true);
}

void aero::interface::AeroMoveitInterfaceDeprecated::moveToAsync(geometry_msgs::Pose _pose)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords;
  tf::poseMsgToEigen(_pose, coords);
  moveTo(coords, true);
}

bool aero::interface::AeroMoveitInterfaceDeprecated::at(std::string _location, double _thre)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  return isAt(_location, _thre);
}

bool aero::interface::AeroMoveitInterfaceDeprecated::at(geometry_msgs::Pose _pose, double _thre)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords;
  tf::poseMsgToEigen(_pose, coords);
  return isAt(coords, _thre);
}

void aero::interface::AeroMoveitInterfaceDeprecated::faceTowardAsync(std::string _location)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  faceToward(_location);
}

void aero::interface::AeroMoveitInterfaceDeprecated::faceTowardAsync(geometry_msgs::Pose _pose)
{
  ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords;
  tf::poseMsgToEigen(_pose, coords);
  faceToward(coords);
}

bool aero::interface::AeroMoveitInterfaceDeprecated::checkMoveTo(geometry_msgs::Pose _pose)
{
  ROS_FATAL_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
  aero::Transform coords;
  tf::poseMsgToEigen(_pose, coords);
  return checkMoveTo(coords);
}
