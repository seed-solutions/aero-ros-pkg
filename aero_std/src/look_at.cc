#include <ros/ros.h>
#include <thread>
#include <mutex>
#include "aero_std/AeroMoveitInterface.hh"
#include <std_srvs/Trigger.h>
#include <aero_startup/AeroSendJoints.h>

class LookAt {
public: LookAt(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _robot)
  : nh_(_nh) {
  robot_ = _robot;

  set_target_ =
    nh_.subscribe("/look_at/set_target_topic", 1, &LookAt::SetTarget, this);

  set_rpy_ =
    nh_.subscribe("/look_at/rpy", 1, &LookAt::SetRPY, this);

  set_static_base_ =
    nh_.subscribe("/look_at/target/static", 1, &LookAt::SetStaticBase, this);

  set_static_base_ =
    nh_.subscribe("/look_at/target/static/map", 1, &LookAt::SetStaticMap, this);

  create_thread_base_ =
    nh_.subscribe("/look_at/target", 1, &LookAt::CreateThreadBase, this);

  create_thread_map_ =
    nh_.subscribe("/look_at/target/map", 1, &LookAt::CreateThreadMap, this);

  sneak_values_ =
    nh_.subscribe("/aero_controller/command", 1, &LookAt::SneakValues, this);

  reply_model_update__ =
    nh_.advertiseService("/look_at/get_model_update", &LookAt::ReplyModelUpdate, this);

  speed_factor_ =
    nh_.subscribe("/aero_controller/speed_overwrite", 1, &LookAt::CheckSpeed, this);

  target_thread_alive_ = false;
  target_thread_kill_ = true;
  prev_msgs_ = "/look_at/manager_disabled";

  map_coordinate_ = false;
  sneak_ = false;
  p0_ = Eigen::Vector3d(0.0, 0.0, 0.0);
};

public: ~LookAt() {};

private: void SetTarget(const std_msgs::String &_msg) {
  ROS_INFO("got %s where prev %s p0: %f %f %f",
           _msg.data.c_str(), prev_msgs_.c_str(), p0_.x(), p0_.y(), p0_.z());

  if (_msg.data == prev_msgs_) // don't do anything if same topic
    return;

  thread_alive_mutex_.lock();
  target_thread_kill_ = true;
  thread_alive_mutex_.unlock();
  // listens to different topic, no need to care about thread state
  sub_.shutdown();
  if (_msg.data.find("/map/") != std::string::npos) {
    ROS_INFO("Listening to %s!!!! in map coordinate", _msg.data.c_str());
    map_coordinate_ = true;
  } else {
    ROS_INFO("Listening to %s!!!! in base coordinate", _msg.data.c_str());
    map_coordinate_ = false;
  }
  sub_ = nh_.subscribe(_msg.data, 1, &LookAt::Callback, this);
  prev_msgs_ = _msg.data;
  sneak_ = (_msg.data == "/look_at/manager_disabled" ? true : false);
};

private: void Callback(const geometry_msgs::Point::ConstPtr &_msg) {
  if (map_coordinate_) {
    // get robot position in map
    Eigen::Vector3d vec_in_base =
      robot_->volatileTransformToBase(_msg->x, _msg->y, _msg->z);
    robot_->setRobotStateToCurrentState();
    robot_->setLookAt(vec_in_base);
  } else {
    robot_->setRobotStateToCurrentState();
    robot_->setLookAt(_msg->x, _msg->y, _msg->z);
  }
  if (factor_ > 0.000001)
    robot_->sendNeckAsync(2000 * factor_);
};

private: void SetRPY(const geometry_msgs::Point::ConstPtr &_msg) {
  thread_alive_mutex_.lock();
  target_thread_kill_ = true;
  thread_alive_mutex_.unlock();
  sub_.shutdown();
  sneak_ = false;
  ROS_INFO("SetRPY: setting neck to %f %f %f", _msg->x, _msg->y, _msg->z);
  robot_->setNeck(_msg->x, _msg->y, _msg->z);
  if (factor_ > 0.000001)
    robot_->sendNeckAsync(2000 * factor_);
};

private: void SetStaticBase(const geometry_msgs::Point::ConstPtr &_msg) {
  thread_alive_mutex_.lock();
  target_thread_kill_ = true;
  thread_alive_mutex_.unlock();
  sub_.shutdown();
  sneak_ = false;
  map_coordinate_ = false;
  Callback(_msg);
};

private: void SetStaticMap(const geometry_msgs::Point::ConstPtr &_msg) {
  thread_alive_mutex_.lock();
  target_thread_kill_ = true;
  thread_alive_mutex_.unlock();
  sub_.shutdown();
  sneak_ = false;
  map_coordinate_ = true;
  Callback(_msg);
};

private: void CreateThreadBase(const geometry_msgs::Point _msg) {
  sneak_ = false;
  map_coordinate_ = false;
  CreateThread(_msg, "/look_at/positioned_target");
};

private: void CreateThreadMap(const geometry_msgs::Point _msg) {
  sneak_ = false;
  map_coordinate_ = true;
  CreateThread(_msg, "/look_at/positioned_target/map/");
};

private: void CreateThread
(const geometry_msgs::Point _msg, const std::string _topic) {
  // check if any thread is already running
  thread_alive_mutex_.lock();
  if (target_thread_alive_) {
    target_thread_kill_ = true;
    thread_alive_mutex_.unlock();
    bool thread_alive = true;
    while (thread_alive) { // wait till current thread is killed
      usleep(100 * 1000);
      thread_alive_mutex_.lock();
      thread_alive = target_thread_alive_;
      thread_alive_mutex_.unlock();
    }
  } else {
    thread_alive_mutex_.unlock();
  }
  sub_.shutdown();
  ROS_INFO("Listening to %s!!!!", _topic.c_str());
  sub_ = nh_.subscribe(_topic, 1, &LookAt::Callback, this);

  // set values
  p_.x = _msg.x;
  p_.y = _msg.y;
  p_.z = _msg.z;
  prev_msgs_ = _topic;

  // build new thread
  target_thread_alive_ = true;
  target_thread_kill_ = false;
  std::thread run([&](double _x, double _y, double _z, std::string _t) {
      ros::Publisher pub =
        nh_.advertise<geometry_msgs::Point>(_t, 1);
      bool thread_kill = false;
      while (!thread_kill) {
        geometry_msgs::Point msg;
        msg.x = _x; msg.y = _y; msg.z = _z;
        pub.publish(msg);
        usleep(800 * 1000);
        thread_alive_mutex_.lock();
        thread_kill = target_thread_kill_;
        thread_alive_mutex_.unlock();
      }
      thread_alive_mutex_.lock();
      target_thread_alive_ = false;
      thread_alive_mutex_.unlock();
    }, _msg.x, _msg.y, _msg.z, _topic);
  run.detach();
}

private: void SneakValues
(const trajectory_msgs::JointTrajectory::ConstPtr &_msg) {
  if (!sneak_)
    return;

  for (size_t i = 0; i < _msg->joint_names.size(); ++i)
    if (_msg->joint_names.at(i) == "neck_r_joint") {
      p0_.x() = _msg->points.back().positions.at(i);
    } else if (_msg->joint_names.at(i) == "neck_p_joint") {
      p0_.y() = _msg->points.back().positions.at(i);
    } else if (_msg->joint_names.at(i) == "neck_y_joint") {
      p0_.z() = _msg->points.back().positions.at(i);
    }
};

private: bool ReplyModelUpdate(aero_startup::AeroSendJoints::Request &_req,
                               aero_startup::AeroSendJoints::Response &_res) {
  _res.points.positions = {p0_.x(), p0_.y(), p0_.z()};
  return true;
};

private: void CheckSpeed(const std_msgs::Float32::ConstPtr &_msg) {
  factor_ = _msg->data;
};

private: ros::NodeHandle nh_;

private: ros::Subscriber set_target_;

private: ros::Subscriber set_rpy_;

private: ros::Subscriber set_static_base_;

private: ros::Subscriber set_static_map_;

private: ros::Subscriber create_thread_base_;

private: ros::Subscriber create_thread_map_;

private: ros::Subscriber sneak_values_;

private: ros::Subscriber speed_factor_;

private: ros::Subscriber sub_;

private: ros::ServiceServer reply_prev_topic_;

private: ros::ServiceServer reply_model_update__;

private: aero::interface::AeroMoveitInterfacePtr robot_;

private: std::mutex thread_alive_mutex_;

private: bool target_thread_alive_;

private: bool target_thread_kill_;

private: bool sneak_;

// neck value for previous == ""
private: Eigen::Vector3d p0_;

// target value for previous == "/look_at/positioned_target"
private: geometry_msgs::Point p_;

private: std::string prev_msgs_;

private: bool map_coordinate_;

private: float factor_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "lookat_manager");
  ros::NodeHandle nh;

  aero::interface::AeroMoveitInterfacePtr robot
    (new aero::interface::AeroMoveitInterface(nh));
  std::shared_ptr<LookAt> lookat(new LookAt(nh, robot));

  ros::spin();

  return 0;
}
