#include "aero_navigation/navigation_local/class/Runner.hh"
#include "aero_navigation/navigation_local/class/RunByTime.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "run_by_time");
  ros::NodeHandle nh;

  aero::navigation::RunByTimePtr time_runner(
      new aero::navigation::RunByTime(nh));

  ros::spin();

  return 0;
}
