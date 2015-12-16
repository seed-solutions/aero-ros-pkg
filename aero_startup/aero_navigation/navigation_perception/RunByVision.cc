#include "aero_navigation/navigation_local/class/Runner.hh"
#include "aero_navigation/navigation_perception/class/RunByVision.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "run_by_vision");
  ros::NodeHandle nh;

  aero::navigation::RunByVisionPtr target_runner(
      new aero::navigation::RunByVision(nh));

  ros::spin();

  return 0;
}
