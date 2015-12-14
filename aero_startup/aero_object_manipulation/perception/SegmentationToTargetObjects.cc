#include "aero_object_manipulation/perception/class/ClusteredCloud.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation_to_target_objects");
  ros::NodeHandle nh;
  aero::common::ClusteredCloudPtr objects(
      new aero::common::ClusteredCloud(nh));

  ros::Rate r(30);

  while (ros::ok())
  {
    ros::spinOnce();
    objects->Spin();
    r.sleep();
  }

  return 0;
}
