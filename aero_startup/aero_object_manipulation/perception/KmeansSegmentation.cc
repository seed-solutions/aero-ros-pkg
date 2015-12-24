#include "aero_object_manipulation/perception/class/KmeansGapClustering.hh"

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "kmeans_segmentation");
  ros::NodeHandle nh;
  aero::common::KmeansGapClusteringPtr kmeans(
      new aero::common::KmeansGapClustering(nh));

  ros::spin();

  return 0;
}

