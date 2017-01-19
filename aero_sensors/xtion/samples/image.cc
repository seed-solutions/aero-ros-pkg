#include <ros/ros.h>
#include <chrono>
#include "aero_sensors/XtionInterface.hh"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xtion_image_sample");
  ros::NodeHandle nh;

  xtion::interface::XtionInterfacePtr xtion
    (new xtion::interface::XtionInterface(nh));

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
