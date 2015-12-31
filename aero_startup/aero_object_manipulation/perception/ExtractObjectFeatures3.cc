#include "aero_object_manipulation/perception/class/Base.hh"
#include "aero_object_manipulation/perception/class/ObjectFeatures.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_object_features_3");
  ros::NodeHandle nh;
  aero::perception::ObjectFeaturesPtr object(
      new aero::perception::ObjectFeatures(nh));

  ros::spin();

  return 0;
}
