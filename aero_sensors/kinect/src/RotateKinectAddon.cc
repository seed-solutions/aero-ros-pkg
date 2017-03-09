#include "aero_sensors/RotateKinectAddon.hh"

using namespace aero;
using namespace addon;

//////////////////////////////////////////////////
RotateKinectAddon::RotateKinectAddon(ros::NodeHandle _nh)
  : nh_(_nh), pseudo_tf_spinner_(1, &pseudo_tf_queue_)
{
  kinect_control_publisher_ = nh_.advertise<std_msgs::Float32>
    ("/kinect_controller/command", 1000);

  base_to_eye_.position.x = 0;
  base_to_eye_.position.y = 0;
  base_to_eye_.position.z = 0;
  base_to_eye_.orientation.x = 1;
  base_to_eye_.orientation.y = 0;
  base_to_eye_.orientation.z = 0;
  base_to_eye_.orientation.w = 0;

  ros::SubscribeOptions pseudo_tf_ops =
    ros::SubscribeOptions::create<geometry_msgs::Pose>
    ("/matrix/base_to_eye",
     100,
     boost::bind(&RotateKinectAddon::SubscribeCameraPseudoTf, this, _1),
     ros::VoidPtr(),
     &pseudo_tf_queue_);

  camera_pseudo_tf_subscriber_ = nh_.subscribe(pseudo_tf_ops);

  pseudo_tf_spinner_.start();
}

//////////////////////////////////////////////////
RotateKinectAddon::~RotateKinectAddon()
{
}

//////////////////////////////////////////////////
void RotateKinectAddon::rotateKinectTo(int _angle)
{
  std_msgs::Float32 angle;
  angle.data = static_cast<float>(_angle);
  kinect_control_publisher_.publish(angle);
}

//////////////////////////////////////////////////
std::pair<Eigen::Vector3f, Eigen::Quaternionf>
RotateKinectAddon::getBaseToEye()
{
  geometry_msgs::Pose res;
  tf_mutex_.lock();
  res = base_to_eye_;
  tf_mutex_.unlock();
  return {
    Eigen::Vector3f(res.position.x,
                    res.position.y,
                    res.position.z),
    Eigen::Quaternionf(res.orientation.x,
                       res.orientation.y,
                       res.orientation.z,
                       res.orientation.w)
    };
}

//////////////////////////////////////////////////
void RotateKinectAddon::SubscribeCameraPseudoTf
(const geometry_msgs::Pose::ConstPtr& _pose)
{
  tf_mutex_.lock();
  base_to_eye_.position.x = _pose->position.x;
  base_to_eye_.position.y = _pose->position.y;
  base_to_eye_.position.z = _pose->position.z;
  base_to_eye_.orientation.x = _pose->orientation.x;
  base_to_eye_.orientation.y = _pose->orientation.y;
  base_to_eye_.orientation.z = _pose->orientation.z;
  base_to_eye_.orientation.w = _pose->orientation.w;
  tf_mutex_.unlock();
}
