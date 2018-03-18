#include <aero_std/AeroMoveitInterface.hh>
#include <thread>
#include <mutex>

/// @file async_neck_control.cc
/// @brief how to control neck independent to body movement
/// external node required: rosrun aero_std look_at.

int main(int argc, char **argv)
{
  // setups for example

  // init ros
  ros::init(argc, argv, "async_neck_control_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));



  // main example starts here

  // reset robot pose
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);

  // trackingMode enables asynchronous control of neck joints
  robot->setTrackingMode(true);

  // neck controlled in some thread
  std::mutex robot_mutex;
  std::thread head_thread([&](){
      int count = 0;
      float yaw = 45.0 * M_PI / 180;
      // move head back and forth for ten seconds
      while (count < 6) {
        robot_mutex.lock();
        robot->setNeck(0.0, 0.0, yaw);
        robot->sendNeckAsync();
        robot_mutex.unlock();
        sleep(2);
        yaw = yaw > 0 ? -45.0 : 45.0;
        yaw *= M_PI / 180;
        ++count;
      }
    });

  sleep(4);

  // change robot pose (head angles should be updated in background)
  ROS_INFO("moving torso"); // send to different pose
  aero::joint_angle_map joints2;
  joints2[aero::joint::waist_p] = 0.524;
  robot_mutex.lock();
  robot->setRobotStateVariables(joints2);
  robot->sendAngleVector(5000);
  robot_mutex.unlock();
  sleep(5);
  head_thread.join();

  // disable asynchronous control of neck joints
  robot->setTrackingMode(false);
  sleep(1); // wait for background thread to completely finish

  // finish
  ROS_INFO("reseting robot pose");
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->sendAngleVector(3000);
  sleep(3);

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
