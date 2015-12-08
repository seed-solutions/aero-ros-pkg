#include "aero_object_manipulation/perception/class/ObjectFeatures.hh"
#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_object_manipulation/perception/class/XYZHSIPointCloud.hh"
#include "aero_navigation/navigation_local/class/Runner.hh"
#include "aero_navigation/navigation_perception/class/RunByVision.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_and_run");
  ros::NodeHandle nh;

  aero::perception::ObjectFeaturesPtr object(
      new aero::perception::ObjectFeatures());
  aero::perception::XYZHSIPointCloudPtr sensor(
      new aero::perception::XYZHSIPointCloud(nh));
  aero::navigation::RunByVisionPtr target_runner(
      new aero::navigation::RunByVision(nh, sensor, object)); 

  while (ros::ok())
  {
    ros::spinOnce();
    object->ExtractObjectFeatures(
	sensor->GetCenter(), sensor->GetVertices(), sensor->GetCloud());
  }

  return 0;
}
