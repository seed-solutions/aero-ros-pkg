#include <aero_std/AeroMoveitInterface.hh>
#include <thread>

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

  // create a poses to look at
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);
  Eigen::Vector3d obj0 = robot->getEEFPosition(aero::arm::rarm, aero::eef::pick);

  ROS_INFO("moving arm"); // send to different pose
  std::map<aero::joint, double> joints;
  joints[aero::joint::r_elbow] = -1.745;
  robot->setRobotStateVariables(joints);
  robot->sendAngleVector(3000);
  sleep(1);
  Eigen::Vector3d obj1 = robot->getEEFPosition(aero::arm::rarm, aero::eef::pick);

  // main example starts here

  // static target example

  // reset robot pose
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);

  // set positioned look at on background
  robot->setTrackingMode(true);
  robot->setLookAt(obj0); // set position to track

  // change robot pose (lookAt should be updated in background)
  ROS_INFO("moving torso"); // send to different pose
  std::map<aero::joint, double> joints2;
  joints2[aero::joint::waist_p] = 0.524;
  robot->setRobotStateVariables(joints2);
  robot->sendAngleVector(5000);
  sleep(1);

  // set background tracking to false
  robot->setTrackingMode(false);

  // streaming target example

  // reset robot pose
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);

  // set position streaming topic name
  robot->setLookAtTopic("/look_at/some_topic"); // set topic

  // dummy position publishing node
  std::thread streamer([&](Eigen::Vector3d _p1, Eigen::Vector3d _p2){
      ros::Publisher streaming_target =
        nh.advertise<geometry_msgs::Point>("/look_at/some_topic", 10);
      int count = 0;
      while (count <= 6) { // keep updating lookAt position every 100ms
        geometry_msgs::Point msg;
        if (count % 2 == 0) {
          msg.x = _p1.x(); msg.y = _p1.y(); msg.z = _p1.z();
        } else {
          msg.x = _p2.x(); msg.y = _p2.y(); msg.z = _p2.z();
        }
        streaming_target.publish(msg);
        usleep(1000 * 1000);
        ++count;
      }
    }, obj0, obj1);

  // change robot pose (lookAt should be updated in background)
  ROS_INFO("moving torso"); // send to different pose
  robot->setRobotStateVariables(joints2);
  robot->sendAngleVector(5000);
  sleep(1);

  // set background tracking to false
  robot->setTrackingMode(false);

  // finish
  ROS_INFO("reseting robot pose");
  robot->sendResetManipPose();
  sleep(1);

  streamer.join();

  ROS_INFO("demo node finished");
  ros::shutdown();
  return 0;
}
