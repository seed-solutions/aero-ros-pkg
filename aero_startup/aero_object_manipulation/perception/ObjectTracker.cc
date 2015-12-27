#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_object_manipulation/perception/class/ObjectTrackerPointCloud.hh"
#include "aero_common/time.h"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_to_target_objects");
  ros::NodeHandle nh;
  aero::perception::ObjectTrackerPointCloudPtr sensor(
      new aero::perception::ObjectTrackerPointCloud(nh));

  ros::Rate r(10);

  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
