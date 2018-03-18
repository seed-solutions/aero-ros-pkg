#include <ros/ros.h>
#include <aero_std/AeroMoveitInterface.hh>
#include <thread>
#include <mutex>
#include "aero_std/time.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_test");
  ros::NodeHandle nh;

  aero::interface::AeroMoveitInterface::Ptr robot
    (new aero::interface::AeroMoveitInterface(nh));
  sleep(1);

  // test 1

  robot->speakAsync("Starting test 1. Linear reset.");

  robot->setInterpolation(aero::interpolation::i_linear);
  robot->sendResetManipPose(3000);

  robot->speak("Complete test 1.", 5.0);

  // test 2

  robot->speakAsync("Starting test 2. Sigmoid interpolation.");

  robot->setInterpolation(aero::interpolation::i_sigmoid);
  aero::joint_angle_map av;
  robot->getResetManipPose(av);
  av[aero::joint::r_shoulder_p] = -89.0 * M_PI / 180;
  av[aero::joint::r_shoulder_r] = -45.0 * M_PI / 180;
  av[aero::joint::r_elbow] = -15.0 * M_PI / 180;
  robot->sendAngleVector(av, 5000);

  robot->sendResetManipPose();
  robot->speak("Complete test 2.", 5.0);

  // test 3

  robot->speakAsync("Starting test 3. Asynchronous control.");

  robot->setInterpolation(aero::interpolation::i_linear);
  robot->setTrackingMode(true);
  std::mutex robot_mutex;
  std::thread head_thread([&](){
      auto start = aero::time::now();
      float yaw = 45.0 * M_PI / 180;
      // move head back and forth for ten seconds
      while (aero::time::ms(aero::time::now() - start) < 12000) {
        std::cout << aero::time::ms(aero::time::now() - start) << std::endl;
        robot_mutex.lock();
        robot->setNeck(0.0, 0.0, yaw);
        robot->sendNeckAsync();
        robot_mutex.unlock();
        sleep(2);
        yaw = yaw > 0 ? -45.0 : 45.0;
        yaw *= M_PI / 180;
      }
    });

  sleep(4);

  robot_mutex.lock();
  robot->sendAngleVectorAsync(av, 5000);
  robot_mutex.unlock();
  sleep(5);
  head_thread.join();

  robot->setTrackingMode(false);
  robot->sendResetManipPose();
  robot->speak("Complete test 3.", 5.0);

  // test 4

  robot->speakAsync("Starting test 4. Interfere control.");

  robot->sendAngleVectorAsync(av, 5000);
  sleep(2);

  // note: sendResetManipPose calls sendJoints, which cannot interrupt
  aero::joint_angle_map av0;
  robot->getResetManipPose(av0);
  robot->setRobotStateVariables(av0);
  robot->sendAngleVector(5000);
  robot->speak("Complete test 4.", 5.0);

  // test 5

  robot->speakAsync("Starting test 5. Stop control.");

  robot->sendAngleVectorAsync(av, 5000);
  usleep(2500 * 1000);

  av[aero::joint::r_shoulder_p] = std::numeric_limits<float>::quiet_NaN();
  av[aero::joint::r_shoulder_r] = std::numeric_limits<float>::quiet_NaN();
  av[aero::joint::r_elbow] = std::numeric_limits<float>::quiet_NaN();

  robot->speakAsync("Holding pose for 5 seconds.");
  robot->sendAngleVector(av, 5000);

  robot->sendResetManipPose();
  robot->speak("Complete test 5.", 5.0);

  return 0;
}
