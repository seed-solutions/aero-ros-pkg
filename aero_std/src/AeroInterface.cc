#include "aero_std/AeroInterface.hh"

using namespace aero;
using namespace interface;

//////////////////////////////////////////////////
AeroInterface::AeroInterface(ros::NodeHandle _nh) : nh_(_nh)
{
  pose_publisher_ = nh_.advertise<trajectory_msgs::JointTrajectory>
    ("/aero_controller/command", 1000);

  waist_client_ = nh_.serviceClient<aero_startup::AeroTorsoController>
    ("/aero_torso_controller");

  hand_client_ = nh_.serviceClient<aero_startup::AeroHandController>
    ("/aero_hand_controller");

  interpolation_client_ = nh_.serviceClient<aero_startup::AeroInterpolation>
    ("/aero_controller/interpolation");

  ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    ("/move_base/goal", true);

  speech_publisher_ = nh_.advertise<std_msgs::String>
    ("/windows/voice", 1000);

  speech_listener_ = nh_.subscribe
    ("/detected/speech/template",  1000, &AeroInterface::Listener, this);

  speech_detection_settings_publisher_ = nh_.advertise<std_msgs::String>
    ("/settings/speech", 1000);

  tts_flag_listener_ = nh_.subscribe
    ("/windows/voice/finished", 1000, &AeroInterface::TTSFlagListener, this);

  mbased_loaded_ =
    nh_.advertiseService("/aero_mbased/loaded", &AeroInterface::MBasedLoaded, this);

  tts_finished_ = false;

  ignore_count_ = 0;

  // don't remove the next line! it's not used but compile fails without it!
  //tf::TransformBroadcaster tmp;

  // robot status

  body_pose_ = ReadOnlyResetManipPose();

  base_position_world_ = Eigen::Vector3f(0, 0, 0);

  // optional

  kinect_control_publisher_ = nh_.advertise<std_msgs::Float32>
    ("/kinect_controller/command", 1000);

#ifdef LINK_VIEWER_ON
  // init viewer
  InitViewer();
#endif
}

//////////////////////////////////////////////////
AeroInterface::~AeroInterface()
{
}

//////////////////////////////////////////////////
void AeroInterface::ResetManipPose()
{
  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names =
  {
    "r_shoulder_p_joint", "r_shoulder_r_joint", "r_shoulder_y_joint",
    "r_elbow_joint",
    "r_wrist_y_joint", "r_wrist_p_joint", "r_wrist_r_joint",
    "l_shoulder_p_joint", "l_shoulder_r_joint", "l_shoulder_y_joint",
    "l_elbow_joint",
    "l_wrist_y_joint", "l_wrist_p_joint", "l_wrist_r_joint",
    "waist_y_joint", "waist_p_joint", "waist_r_joint",
    "neck_y_joint", "neck_p_joint", "neck_r_joint"
  };
  msg.points[0].positions.resize(20);
  msg.points[0].positions =
  {
    -0.24434609527920603, 0.0, 0.0,
    -2.356194490192344,
    0.0, 0.0, 0.0,
    -0.24434609527920603, 0.0, 0.0,
    -2.356194490192344,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0
  };
  msg.points[0].time_from_start = ros::Duration(2.0);
  usleep(1000 * 1000);
  pose_publisher_.publish(msg);
  ROS_WARN("resetting pose");
  usleep(3000 * 1000);
  ROS_WARN("resetting done");
}

//////////////////////////////////////////////////
bool AeroInterface::MoveWaist(Eigen::Vector3f _vector, std::string _coordinate)
{
  aero_startup::AeroTorsoController srv;
  srv.request.x = _vector.x();
  srv.request.z = _vector.z();
  srv.request.coordinate = _coordinate;

  if (!waist_client_.call(srv)) {
    ROS_ERROR("move waist failed service call");
    return false;
  }

  if (srv.response.status == "success") {
    usleep(static_cast<int>(srv.response.time_sec * 1000) * 1000);
    base_position_world_ = Eigen::Vector3f(srv.response.x, 0, srv.response.z);
    return true;
  }

  ROS_ERROR("%s", srv.response.status.c_str());
  return false;
}

//////////////////////////////////////////////////
bool AeroInterface::OpenHand(bool _yes, std::string _arm)
{
  return OpenHand(_yes, _arm, -0.9, 0.8);
}

//////////////////////////////////////////////////
bool AeroInterface::OpenHand(bool _yes, std::string _arm,
                             float _warn, float _fail)
{
  aero_startup::AeroHandController srv;
  srv.request.hand = _arm;
  srv.request.thre_warn = _warn;
  srv.request.thre_fail = _fail;
  if (_yes) srv.request.command = "ungrasp";
  else srv.request.command = "grasp";
  
  if (!hand_client_.call(srv)) {
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
bool AeroInterface::OpenHand(float _angle, std::string _arm)
{
  return OpenHand(_angle, _arm, -0.9, 0.8);
}

//////////////////////////////////////////////////
bool AeroInterface::OpenHand(float _angle, std::string _arm,
                             float _warn, float _fail)
{
  aero_startup::AeroHandController srv;
  srv.request.hand = _arm;
  srv.request.thre_warn = _warn;
  srv.request.thre_fail = _fail;
  srv.request.command = "grasp-angle";
  srv.request.larm_angle = _angle;
  srv.request.rarm_angle = _angle;

  if (!hand_client_.call(srv)) {
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
bool AeroInterface::GoPos(float _x, float _y, float _theta)
{
  return GoPos(_x, _y, _theta, 5.0);
}

//////////////////////////////////////////////////
bool AeroInterface::GoPos(float _x, float _y, float _theta,
                          float _time_out)
{
  // mm -> meters
  geometry_msgs::Point p;
  p.x = _x * 0.001;
  p.y = _y * 0.001;
  p.z = 0;

  // deg -> rad
  geometry_msgs::Quaternion q;
  q.x = sin(_theta * 0.008726646259971648);
  q.y = 0;
  q.z = 0;
  q.w = cos(_theta * 0.008726646259971648);

  geometry_msgs::Pose pose;
  pose.position = p;
  pose.orientation = q;

  geometry_msgs::PoseStamped pstamp;
  pstamp.pose = pose;

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose = pstamp;

  ac_->sendGoal(goal);
  return ac_->waitForResult(ros::Duration(_time_out));
}

//////////////////////////////////////////////////
void AeroInterface::Speak(std::string _speech)
{
  std_msgs::String msg;
  msg.data = _speech;
  speech_publisher_.publish(msg);
  while (!tts_finished_)
    ros::spinOnce();
  tts_finished_ = false;
}

//////////////////////////////////////////////////
void AeroInterface::SpeakAsync(std::string _speech)
{
  std_msgs::String msg;
  msg.data = _speech;
  speech_publisher_.publish(msg);
  ++ignore_count_;
}

//////////////////////////////////////////////////
void AeroInterface::Speak(std::string _speech, float _wait_sec)
{
  std_msgs::String msg;
  msg.data = _speech;
  speech_publisher_.publish(msg);
  usleep(static_cast<int>(_wait_sec * 1000) * 1000);
  ++ignore_count_;
}

//////////////////////////////////////////////////
void AeroInterface::SetInterpolation(const int _intrpl_type)
{
  aero_startup::AeroInterpolation srv;
  srv.request.type.push_back(_intrpl_type);

  if (!interpolation_client_.call(srv)) {
    ROS_WARN("interpolation failed service call");
  }
}

//////////////////////////////////////////////////
void AeroInterface::SetInterpolation(aero::interpolation::settings _intrpl)
{
  aero_startup::AeroInterpolation srv;
  srv.request = _intrpl;

  if (!interpolation_client_.call(srv)) {
    ROS_WARN("interpolation failed service call");
  }
}

//////////////////////////////////////////////////
void AeroInterface::SendAngleVector
(aero_msgs::JointAngles _av, geometry_msgs::Vector3 _look_at,
 int _time_ms, bool _wait_interpolation)
{
  body_pose_ = _av; // save body pose

  ConvertRad(_av); // convert degree to radian

  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names =
  {
    "r_shoulder_p_joint", "r_shoulder_r_joint", "r_shoulder_y_joint",
    "r_elbow_joint",
    "r_wrist_y_joint", "r_wrist_p_joint", "r_wrist_r_joint",
    "l_shoulder_p_joint", "l_shoulder_r_joint", "l_shoulder_y_joint",
    "l_elbow_joint",
    "l_wrist_y_joint", "l_wrist_p_joint", "l_wrist_r_joint",
    "waist_y_joint", "waist_p_joint", "waist_r_joint",
    "neck_y_joint", "neck_p_joint", "neck_r_joint"
  };
  msg.points[0].positions.resize(20);
  msg.points[0].positions =
  {
    _av.r_shoulder_p, _av.r_shoulder_r, _av.r_shoulder_y,
    _av.r_elbow,
    _av.r_wrist_y, _av.r_wrist_p, _av.r_wrist_r,
    _av.l_shoulder_p, _av.l_shoulder_r, _av.l_shoulder_y,
    _av.l_elbow,
    _av.l_wrist_y, _av.l_wrist_p, _av.l_wrist_r,
    _av.waist_y, _av.waist_p, _av.waist_r,
    0.017453292519943295 * _look_at.z,
    0.017453292519943295 * _look_at.y,
    0.017453292519943295 * _look_at.x
  };
  msg.points[0].time_from_start = ros::Duration(_time_ms * 0.001);
  usleep(1000 * 1000);
  pose_publisher_.publish(msg);

  if (_wait_interpolation)
    usleep(_time_ms * 1000);
}

//////////////////////////////////////////////////
void AeroInterface::SendAngleVector
(aero_msgs::JointAngles _av, int _time_ms, bool _wait_interpolation)
{
  body_pose_ = _av; // save body pose

  ConvertRad(_av); // convert degree to radian

  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names =
  {
    "r_shoulder_p_joint", "r_shoulder_r_joint", "r_shoulder_y_joint",
    "r_elbow_joint",
    "r_wrist_y_joint", "r_wrist_p_joint", "r_wrist_r_joint",
    "l_shoulder_p_joint", "l_shoulder_r_joint", "l_shoulder_y_joint",
    "l_elbow_joint",
    "l_wrist_y_joint", "l_wrist_p_joint", "l_wrist_r_joint",
    "waist_y_joint", "waist_p_joint", "waist_r_joint"
  };
  msg.points[0].positions.resize(17);
  msg.points[0].positions =
  {
    _av.r_shoulder_p, _av.r_shoulder_r, _av.r_shoulder_y,
    _av.r_elbow,
    _av.r_wrist_y, _av.r_wrist_p, _av.r_wrist_r,
    _av.l_shoulder_p, _av.l_shoulder_r, _av.l_shoulder_y,
    _av.l_elbow,
    _av.l_wrist_y, _av.l_wrist_p, _av.l_wrist_r,
    _av.waist_y, _av.waist_p, _av.waist_r
  };
  msg.points[0].time_from_start = ros::Duration(_time_ms * 0.001);
  usleep(1000 * 1000);
  pose_publisher_.publish(msg);

  if (_wait_interpolation)
    usleep(_time_ms * 1000);
}

//////////////////////////////////////////////////
void AeroInterface::SendAngleVector
(geometry_msgs::Vector3 _look_at, int _time_ms, bool _wait_interpolation)
{
  trajectory_msgs::JointTrajectory msg;
  msg.points.resize(1);
  msg.joint_names = {"neck_y_joint", "neck_p_joint", "neck_r_joint"};
  msg.points[0].positions.resize(3);
  msg.points[0].positions =
  {
    0.017453292519943295 * _look_at.z,
    0.017453292519943295 * _look_at.y,
    0.017453292519943295 * _look_at.x
  };
  msg.points[0].time_from_start = ros::Duration(_time_ms * 0.001);
  usleep(1000 * 1000);
  pose_publisher_.publish(msg);

  if (_wait_interpolation)
    usleep(_time_ms * 1000);
}

//////////////////////////////////////////////////
void AeroInterface::SendAngleVector
(aero_msgs::JointAngles _av, geometry_msgs::Vector3 _look_at,
 int _time_ms, aero::interpolation::settings _intrpl,
 bool _wait_interpolation)
{
  aero_startup::AeroInterpolation srv;
  srv.request = _intrpl;

  if (!interpolation_client_.call(srv)) {
    ROS_WARN("interpolation failed service call");
  }

  SendAngleVector(_av, _look_at, _time_ms, _wait_interpolation);
}

//////////////////////////////////////////////////
void AeroInterface::SendAngleVector
(aero_msgs::JointAngles _av, int _time_ms,
 aero::interpolation::settings _intrpl, bool _wait_interpolation)
{
  aero_startup::AeroInterpolation srv;
  srv.request = _intrpl;

  if (!interpolation_client_.call(srv)) {
    ROS_WARN("interpolation failed service call");
  }

  SendAngleVector(_av, _time_ms, _wait_interpolation);
}

//////////////////////////////////////////////////
void AeroInterface::SendAngleVector
(geometry_msgs::Vector3 _look_at, int _time_ms,
 aero::interpolation::settings _intrpl, bool _wait_interpolation)
{
  aero_startup::AeroInterpolation srv;
  srv.request = _intrpl;

  if (!interpolation_client_.call(srv)) {
    ROS_WARN("interpolation failed service call");
  }

  SendAngleVector(_look_at, _time_ms, _wait_interpolation);
}

//////////////////////////////////////////////////
aero_msgs::JointAngles AeroInterface::ReadOnlyResetManipPose()
{
  aero_msgs::JointAngles reset_pose;
  reset_pose.r_shoulder_p = -14.0;
  reset_pose.r_shoulder_r = 0.0;
  reset_pose.r_shoulder_y = 0.0;
  reset_pose.r_elbow = -135.0;
  reset_pose.r_wrist_y = 0.0;
  reset_pose.r_wrist_p = 0.0;
  reset_pose.r_wrist_r = 0.0;
  reset_pose.l_shoulder_p = -14.0;
  reset_pose.l_shoulder_r = 0.0;
  reset_pose.l_shoulder_y = 0.0;
  reset_pose.l_elbow = -135.0;
  reset_pose.l_wrist_y = 0.0;
  reset_pose.l_wrist_p = 0.0;
  reset_pose.l_wrist_r = 0.0;
  reset_pose.waist_y = 0.0;
  reset_pose.waist_p = 0.0;
  reset_pose.waist_r = 0.0;
  return reset_pose;
}

//////////////////////////////////////////////////
aero_msgs::GraspIK::Response AeroInterface::ReadOnlyGraspIK
(aero_msgs::GraspIK::Request _req)
{
#ifdef LINK_VIEWER_ON
  // draw grasp points on viewer
  ViewGrasp(_req);
#endif

  // check mm conversion, abort if not
  if (_req.solve_midpoint) {
    for (unsigned int i = 0; i < _req.mid_pos.size(); ++i)
      if (_req.mid_pos[i].z < 0.2)
        std::runtime_error("Dangerous value in GraspIK! Forget mm conversion?");
  }
  for (unsigned int i = 0; i < _req.end_pos.size(); ++i)
    if (_req.end_pos[i].z < 0.2)
        std::runtime_error("Dangerous value in GraspIK! Forget mm conversion?");

  aero_msgs::GraspIK srv;
  srv.request = _req;

  if (!grasp_client_.call(srv)) {
    srv.response.status = "communication failed";
  }

  return srv.response;
}

//////////////////////////////////////////////////
geometry_msgs::Vector3 AeroInterface::ReadOnlyLookIK
(Eigen::Vector3f _look_at, aero_msgs::JointAngles _av)
{
  aero_msgs::LookIK srv;
  srv.request.look_at_mm.x = _look_at.x();
  srv.request.look_at_mm.y = _look_at.y();
  srv.request.look_at_mm.z = _look_at.z();
  srv.request.body_pose = _av;

  if (!look_client_.call(srv)) {
    geometry_msgs::Vector3 null;
    return null;
  }

  return srv.response.rpy;
}

//////////////////////////////////////////////////
void AeroInterface::StartGraspIKClient()
{
  // initialize grasp_lib load checker
  loaded_grasp_lib_ = false;

  ROS_WARN("waiting for grasp_lib load ...");

  // wait for grasp_lib load
  while (!loaded_grasp_lib_) {
    ros_spin_mutex_.lock();
    ros::spinOnce();
    usleep(500 * 1000);
    ros_spin_mutex_.unlock();
  }

  ROS_WARN("loaded grasp_lib!");

  // initialize grasp client
  grasp_client_ =
    nh_.serviceClient<aero_msgs::GraspIK>("/robot/graspik/readonly");
}

//////////////////////////////////////////////////
void AeroInterface::StartLookAtClient()
{
  // initialize look_lib load checker
  loaded_look_lib_ = false;

  ROS_WARN("waiting for look_lib load ...");

  // wait for look_lib load
  while (!loaded_look_lib_) {
    ros_spin_mutex_.lock();
    ros::spinOnce();
    usleep(500 * 1000);
    ros_spin_mutex_.unlock();
  }

  ROS_WARN("loaded look_lib!");

  // initialize look_lib client
  look_client_ = 
    nh_.serviceClient<aero_msgs::LookIK>("/robot/lookik/readonly");
}

//////////////////////////////////////////////////
aero_msgs::JointAngles AeroInterface::ReverseJointAngles(aero_msgs::JointAngles& _av)
{
  aero_msgs::JointAngles av;
  av = _av;

  av.r_shoulder_p = _av.l_shoulder_p;
  av.r_shoulder_r = -_av.l_shoulder_r;
  av.r_shoulder_y = -_av.l_shoulder_y;
  av.r_elbow = _av.l_elbow;
  av.r_wrist_y = -_av.l_wrist_y;
  av.r_wrist_p = _av.l_wrist_p;
  av.r_wrist_r = -_av.l_wrist_r;
  av.l_shoulder_p = _av.r_shoulder_p;
  av.l_shoulder_r = -_av.r_shoulder_r;
  av.l_shoulder_y = -_av.r_shoulder_y;
  av.l_elbow = _av.r_elbow;
  av.l_wrist_y = -_av.r_wrist_y;
  av.l_wrist_p = _av.r_wrist_y;
  av.l_wrist_r = -_av.r_wrist_y;
  av.waist_r *= -1;
  av.waist_y *= -1;
  return av;
}

//////////////////////////////////////////////////
void AeroInterface::Listener(const std_msgs::String::ConstPtr& _msg)
{
  detected_speech_ = _msg->data;
}

//////////////////////////////////////////////////
void AeroInterface::TTSFlagListener(const std_msgs::String::ConstPtr& _msg)
{
  if (ignore_count_ == 0) tts_finished_ = true;
  else if (ignore_count_ > 0) --ignore_count_;
}

//////////////////////////////////////////////////
bool AeroInterface::MBasedLoaded(aero_msgs::MBasedLoaded::Request& _req,
                                 aero_msgs::MBasedLoaded::Response& _res)
{
  if (_req.data == "look_lib") loaded_look_lib_ = true;
  else if (_req.data == "grasp_lib") loaded_grasp_lib_ = true;
  return true;
}

//////////////////////////////////////////////////
void AeroInterface::ConvertRad(aero_msgs::JointAngles& _av)
{
  _av.r_shoulder_p = 0.017453292519943295 * _av.r_shoulder_p;
  _av.r_shoulder_r = 0.017453292519943295 * _av.r_shoulder_r;
  _av.r_shoulder_y = 0.017453292519943295 * _av.r_shoulder_y;
  _av.r_elbow = 0.017453292519943295 * _av.r_elbow;
  _av.r_wrist_y = 0.017453292519943295 * _av.r_wrist_y;
  _av.r_wrist_p = 0.017453292519943295 * _av.r_wrist_p;
  _av.r_wrist_r = 0.017453292519943295 * _av.r_wrist_r;
  _av.l_shoulder_p = 0.017453292519943295 * _av.l_shoulder_p;
  _av.l_shoulder_r = 0.017453292519943295 * _av.l_shoulder_r;
  _av.l_shoulder_y = 0.017453292519943295 * _av.l_shoulder_y;
  _av.l_elbow = 0.017453292519943295 * _av.l_elbow;
  _av.l_wrist_y = 0.017453292519943295 * _av.l_wrist_y;
  _av.l_wrist_p = 0.017453292519943295 * _av.l_wrist_p;
  _av.l_wrist_r = 0.017453292519943295 * _av.l_wrist_r;
  _av.waist_y = 0.017453292519943295 * _av.waist_y;
  _av.waist_p = 0.017453292519943295 * _av.waist_p;
  _av.waist_r = 0.017453292519943295 * _av.waist_r;
}

//////////////////////////////////////////////////
void AeroInterface::RotateKinectTo(float _angle)
{
  std_msgs::Float32 angle;
  angle.data = _angle;
  kinect_control_publisher_.publish(angle);
}
