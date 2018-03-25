
#include <aero_std/AeroBaseCommander.hh>

aero::base_commander::AeroBaseCommander::AeroBaseCommander(ros::NodeHandle &_nh)
{
  cmd_vel_publisher_ = _nh.advertise<geometry_msgs::Twist>
    ("/cmd_vel", 1000);

  get_spot_ = _nh.serviceClient<aero_std::GetSpot>
    ("/get_spot");

  check_move_to_ = _nh.serviceClient<nav_msgs::GetPlan>
    ("/make_plan");

  // action client
  base_ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    ("/move_base", true);
}

bool aero::base_commander::AeroBaseCommander::getCurrentCoords(aero::Transform &_pose, const std::string &_origin_frame)
{
  tf::StampedTransform tr;
  try {
    listener_.waitForTransform(_origin_frame, robot_base_frame, ros::Time(0), ros::Duration(5.0));
    listener_.lookupTransform (_origin_frame, robot_base_frame, ros::Time(0), tr);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    //ros::Duration(1.0).sleep(); // not need
    return false;
  }

  aero::Transform pose;
#if 0
  geometry_msgs::Pose msg;
  auto pos = tr.getOrigin();
  auto rot = tr.getRotation();

  msg.position.x = pos.x();
  msg.position.y = pos.y();
  msg.position.z = pos.z();

  msg.orientation.w = rot.w();
  msg.orientation.x = rot.x();
  msg.orientation.y = rot.y();
  msg.orientation.z = rot.z();
#endif
  return true;
}

//////////////////////////////////////////////////
bool aero::base_commander::AeroBaseCommander::getLocationCoords(aero::Transform &_pose, const std::string &_location)
{
  aero_std::GetSpot gs;
  gs.request.name = _location;
  get_spot_.call(gs);
#if 0
  geometry_msgs::Pose pose = gs.response.pose;
  return pose;
#endif
  return true;
}

//////////////////////////////////////////////////
bool aero::base_commander::AeroBaseCommander::goPos(double _x,double _y, double _rad, int _timeout_ms, bool _async)
{
  // if turn only, move without path planning
  // if (_x == 0.0 && _y == 0.0) return goPosTurnOnly_(_rad, _timeout_ms);

  aero::Quaternion qua(aero::AngleAxis(_rad, aero::Vector3::UnitZ()));
  aero::Transform a_pose = aero::Translation(_x, _y, 0.0) * qua;
  // a_pose -> pose
  geometry_msgs::Pose pose;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "/base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = pose;
  destination_coords_ = a_pose;

  ROS_INFO("Sending goal");
  base_ac_->sendGoal(goal);

  if (!_async) {
    bool finished_before_timeout = base_ac_->waitForResult(ros::Duration((_timeout_ms + 50) * 0.001));

    if (!finished_before_timeout) {
      ROS_WARN("go pos action didn't finish before timeout %d ms", _timeout_ms);
      return false;
    }
  }

  return true;
}

//////////////////////////////////////////////////
void aero::base_commander::AeroBaseCommander::moveTo(const std::string &_location, bool _async)
{
  aero::Transform coords;
  bool exists = getLocationCoords(coords, _location);
  if (exists) {
    moveTo(coords, _async);
  } else {
    ROS_ERROR("location(%s) is not found", _location.c_str());
  }
}

//////////////////////////////////////////////////
void aero::base_commander::AeroBaseCommander::moveTo(const aero::Transform &_coords, bool _async)
{
  aero::Vector3 pos = _coords.translation();
  geometry_msgs::Pose pose;
  pose.position.x = pos.x();
  pose.position.y = pos.y();
  pose.position.z = pos.z();
  pose.orientation.x = pose.orientation.y = pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "/map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = pose;

  destination_coords_ = _coords;

  ROS_INFO("Sending goal");
  base_ac_->sendGoal(goal);
}

//////////////////////////////////////////////////
bool aero::base_commander::AeroBaseCommander::isMoving()
{
  auto state = base_ac_->getState();
  bool finished = state.isDone();
  return !finished;// moving != finished
}

//////////////////////////////////////////////////
bool aero::base_commander::AeroBaseCommander::isAt(const std::string &_location, double _thre)
{
  aero::Transform loc;
  if(!getLocationCoords(loc, _location)) {
    return false;
  }

  return isAt(loc, _thre);
}

//////////////////////////////////////////////////
bool aero::base_commander::AeroBaseCommander::isAt(const aero::Transform &_pose, double _thre)
{
  aero::Transform res;
  if(!getCurrentCoords(res)) {
    return false;
  }

  aero::Vector3 a(res.translation());
  aero::Vector3 b(_pose.translation());
  double diff = (a - b).norm();

  if (diff > _thre) return false;

  return true;
}

//////////////////////////////////////////////////
void aero::base_commander::AeroBaseCommander::stop()
{
  base_ac_->cancelGoal();
}

//////////////////////////////////////////////////
void aero::base_commander::AeroBaseCommander::go()
{
  moveTo(destination_coords_, true);
}

//////////////////////////////////////////////////
// distanceToDestination
double aero::base_commander::AeroBaseCommander::toDestination(const std::string &_location)
{
  aero::Transform loc;
  getLocationCoords(loc, _location);
  aero::Transform cur;
  getCurrentCoords(cur);

  double dis = 0.0; //TODO distance loc, cur
  return dis;
}

//////////////////////////////////////////////////
void aero::base_commander::AeroBaseCommander::faceTowardAsync(const std::string &_location)
{
  aero::Transform loc;
  getLocationCoords(loc, _location);
  faceTowardAsync(loc);
}

//////////////////////////////////////////////////
void aero::base_commander::AeroBaseCommander::faceTowardAsync(const aero::Transform &_pose)
{
#if 0
  geometry_msgs::Pose cur, pose;
  cur = getCurrentCoords();
  pose.position = cur.position;


  double yaw = std::atan2(cur.position.y - _pose.position.y, cur.position.x -  _pose.position.x) + M_PI;

  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  Eigen::Quaterniond qua = Eigen::Quaterniond(Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
  tf::quaternionEigenToMsg(qua, pose.orientation);

  moveToAsync(pose);
#endif
}

//////////////////////////////////////////////////
bool aero::base_commander::AeroBaseCommander::checkMoveTo(const aero::Transform &_pose) {
  // whether snoid can move to _pose or cannot
#if 0
  nav_msgs::GetPlan srv;
  geometry_msgs::PoseStamped cur,pose;
  geometry_msgs::Pose result;
  ros::Time now = ros::Time::now();
  double tolerance = 0.02;
  cur.header.stamp = now;
  cur.pose = getCurrentCoords();
  pose.header.stamp = now;
  pose.pose = _pose;

  srv.request.start = cur;
  srv.request.goal = pose;
  srv.request.tolerance = tolerance;

  check_move_to_.call(srv);
  ROS_INFO("current pose %f %f %f", cur.pose.position.x, cur.pose.position.y, cur.pose.position.z);
  ROS_INFO("target pose %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  if (srv.response.plan.poses.size() == 0) {
    ROS_INFO("this path failed");
    return false;
  }

  result = srv.response.plan.poses.back().pose;
  double distance = std::sqrt(std::pow(result.position.x - _pose.position.x, 2.0) + std::pow(result.position.y - _pose.position.y, 2.0));
  if (distance > tolerance) {
    ROS_INFO("this path failed");
    return false;
  }
#endif
  ROS_INFO("plan found");
  return true;
}

#if 0
//////////////////////////////////////////////////
bool aero::base_commander::AeroBaseCommander::goPosTurnOnly_(double _rad, int _timeout_ms)
{
  double tolerance = 5.0 * M_PI / 180.0;
  double vel = 0.5; // absolute rotate velocity
  int hz = 30;// cmd_vel is sent with this frewquency
  geometry_msgs::Pose initial_pose = getCurrentCoords();
  double z_ref = 2.0 * acos(initial_pose.orientation.w) + _rad;

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  if (_rad > 0.0) cmd.angular.z = -vel;
  else cmd.angular.z = vel;

  ros::Time now = ros::Time::now();
  ros::Duration limit = ros::Duration(_timeout_ms * 0.001);
  ros::Rate r(hz);
  while (ros::ok()) {
    if (ros::Time::now() - now > limit) break;
    geometry_msgs::Pose cur = getCurrentCoords();
    double z_now = 2.0 * acos(cur.orientation.w);
    double z_diff = z_ref - z_now;
    // -M_PI < z_diff < M_PI
    while (z_diff > M_PI) z_diff -= M_PI * 2.0;
    while (z_diff < -M_PI) z_diff += M_PI * 2.0;

    if (std::abs(z_diff) < tolerance) return true;// finish if diff is lower than tolerance

    // send cmd_vel
    if (z_diff > 0.0) cmd.angular.z = vel;
    else cmd.angular.z = -vel;
    cmd_vel_publisher_.publish(cmd);

    r.sleep();
  }

  ROS_WARN("goPos turn only didn't finish before timeout %d ms", _timeout_ms);
  return false;
}
#endif
