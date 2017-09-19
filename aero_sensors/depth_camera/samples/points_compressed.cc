#include <ros/ros.h>
#include <chrono>
#include "aero_sensors/DepthCameraInterface.hh"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xtion_points_sample");
  ros::NodeHandle nh;

  depth_camera::interface::DepthCameraInterfacePtr xtion
    (new depth_camera::interface::DepthCameraInterface(nh));

  ros::Publisher points_publisher =
    nh.advertise<sensor_msgs::PointCloud2>("/xtion/pointstream", 1);

  ros::Rate r(1);
  while (ros::ok()) {
    auto start = std::chrono::high_resolution_clock::now();

    auto points = xtion->ReadPoints(0.25, 0.25);

    ROS_INFO("finished read points %f",
             static_cast<float>
             (std::chrono::duration_cast<std::chrono::milliseconds>
              (std::chrono::high_resolution_clock::now() - start).count()));
    points_publisher.publish(points);

    r.sleep();
  }
}
