#include "aero_object_manipulation/perception/class/Base.hh"
#include "aero_object_manipulation/perception/class/Kmeans2DGapClustering.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "kmeans_segmentation");
  ros::NodeHandle nh;
  aero::common::KmeansGapClusteringPtr kmeans(
      new aero::common::KmeansGapClustering(nh));

  // ros::spin();

  ros::Rate r(1);

  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}

