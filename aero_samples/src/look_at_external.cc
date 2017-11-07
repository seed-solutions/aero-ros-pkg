#include <aero_std/AeroMoveitInterface.hh>
#include <thread>
#include <mutex>

/// @file look_at_external.cc
/// @brief how to control neck with lookAt manager
/// external node required: rosrun aero_std look_at.

int main(int argc, char **argv)
{
  // init ros
  ros::init(argc, argv, "look_at_external_sample_node");
  ros::NodeHandle nh;
  
  // init robot interface
  aero::interface::AeroMoveitInterfacePtr robot(new aero::interface::AeroMoveitInterface(nh));

  // start lookAt manager
  robot->setLookAtTopic("/look_at/target"); // set topic 

  std::mutex robot_mutex;
  std::thread head_control([&](){
      int count = 0;
      while (count <= 30) { // keep updating lookAt position every 100ms
        robot_mutex.lock();
        // look at right hand position
        Eigen::Vector3d obj = robot->getEEFPosition(aero::arm::rarm, aero::eef::pick);
        robot->setLookAt(obj); // stream values to "/look_at/target"
        if (robot->getLookAtTopic() != "/look_at/target") // in disable mode
          ++count; // prepare thread end only when in disable mode
        robot_mutex.unlock();
        usleep(100 * 1000);
      }
    });

  ROS_INFO("reseting robot pose");
  robot_mutex.lock();
  robot->sendResetManipPose();
  robot_mutex.unlock();
  sleep(1);

  ROS_INFO("moving arm"); // send to different pose
  robot_mutex.lock();
  std::map<aero::joint, double> joints;
  robot->getRobotStateVariables(joints);
  joints[aero::joint::r_elbow] = -1.745;
  robot->setRobotStateVariables(joints);
  robot->sendAngleVectorAsync(3000);
  robot_mutex.unlock();
  sleep(6);

  robot->setLookAtTopic("/look_at/dummy"); // disable threaded lookAt
  // robot should stay looking at current position

  ROS_INFO("reseting robot pose");
  robot_mutex.lock();
  robot->sendResetManipPose();
  robot_mutex.unlock();
  sleep(1);

  robot->setLookAtTopic(""); // re-enable head control from main
  // robot should reset head position

  ROS_INFO("reseting robot pose");
  robot_mutex.lock();
  robot->sendResetManipPose();
  robot_mutex.unlock();
  sleep(1);

  head_control.join();

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
