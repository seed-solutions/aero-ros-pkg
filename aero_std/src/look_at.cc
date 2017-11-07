#include <ros/ros.h>
#include <thread>
#include <mutex>
#include "aero_std/AeroMoveitInterface.hh"

class LookAt {
public: LookAt(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _robot)
  : nh_(_nh) {
  robot_ = _robot;

  set_target_ =
    nh_.subscribe("/look_at/set_target_topic", 1, &LookAt::SetTarget, this);

  create_thread_ =
    nh_.subscribe("/look_at/target", 1, &LookAt::CreateThread, this);

  target_thread_alive_ = false;
  target_thread_kill_ = true;
};

public: ~LookAt() {};

public: void SetTarget(const std_msgs::String::ConstPtr &_msg) {
  thread_alive_mutex_.lock();
  target_thread_kill_ = true;
  thread_alive_mutex_.unlock();
  // listens to different topic, no need to care about thread state
  sub_.shutdown();
  sub_ = nh_.subscribe(_msg->data, 1, &LookAt::Callback, this);
};

public: void Callback(const geometry_msgs::Point::ConstPtr &_msg) {
  robot_->setRobotStateToCurrentState();
  robot_->setLookAt(_msg->x, _msg->y, _msg->z);
  robot_->sendNeckAsync();
};

public: void CreateThread(const geometry_msgs::Point::ConstPtr &_msg) {
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
  sub_ = nh_.subscribe("/look_at/positioned_target", 1, &LookAt::Callback, this);

  // build new thread
  target_thread_alive_ = true;
  target_thread_kill_ = false;
  std::thread run([&](double _x, double _y, double _z){
      ros::Publisher pub =
        nh_.advertise<geometry_msgs::Point>("/look_at/positioned_target", 1);
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
    }, _msg->x, _msg->y, _msg->z);
  run.detach();
}

private: ros::NodeHandle nh_;

private: ros::Subscriber set_target_;

private: ros::Subscriber create_thread_;

private: ros::Subscriber sub_;

private: aero::interface::AeroMoveitInterfacePtr robot_;

private: std::mutex thread_alive_mutex_;

private: bool target_thread_alive_;

private: bool target_thread_kill_;
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
