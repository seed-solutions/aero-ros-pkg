/// @brief sample for depth_camera image capturing
/// @author Kazuhiro Sasabuchi

#include <ros/ros.h>
#include <chrono>
#include "aero_sensors/DepthCameraInterface.hh"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xtion_image_sample");
  ros::NodeHandle nh;

  depth_camera::interface::DepthCameraInterfacePtr xtion
    (new depth_camera::interface::DepthCameraInterface(nh));

  ros::Publisher image_publisher =
    nh.advertise<sensor_msgs::Image>("/xtion/pixelstream", 1);

  ros::Rate r(1);
  while (ros::ok()) {
    auto start = std::chrono::high_resolution_clock::now();

    auto image = xtion->ReadImage();

    ROS_INFO("finished read image %f",
             static_cast<float>
             (std::chrono::duration_cast<std::chrono::milliseconds>
              (std::chrono::high_resolution_clock::now() - start).count()));

    image_publisher.publish(image);
    r.sleep();
  }
}
