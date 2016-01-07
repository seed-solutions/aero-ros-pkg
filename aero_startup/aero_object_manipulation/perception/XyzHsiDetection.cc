#include "aero_object_manipulation/perception/class/Base.hh"
#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_object_manipulation/perception/class/XYZHSIPointCloud.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "xyz_hsi_detection");
  ros::NodeHandle nh;
  aero::perception::XYZHSIPointCloudPtr sensor(
      new aero::perception::XYZHSIPointCloud(nh));

  ros::spin();

  return 0;
}
