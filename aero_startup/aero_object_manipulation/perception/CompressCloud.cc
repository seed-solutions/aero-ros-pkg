#include "aero_object_manipulation/perception/class/Base.hh"
#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_object_manipulation/perception/class/VoxelPointCloud.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "compress_point_cloud");
  ros::NodeHandle nh;
  aero::perception::VoxelPointCloudPtr sensor(
      new aero::perception::VoxelPointCloud(nh));

  ros::spin();

  return 0;
}
