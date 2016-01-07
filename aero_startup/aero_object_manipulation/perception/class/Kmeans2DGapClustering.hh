#ifndef _AERO_COMMON_KMEANS_GAP_CLUSTERING_H_
#define _AERO_COMMON_KMEANS_GAP_CLUSTERING_H_

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include "aero_object_manipulation/perception/class/Base.hh"
#include "aero_common/types.h"
#include "aero_common/time.h"

namespace aero
{
  namespace common
  {

    class KmeansGapClustering : public Base
    {
    private: struct point
	{ Eigen::Vector2f pos; int group; Eigen::Vector3f vertex; };

    public: explicit KmeansGapClustering(ros::NodeHandle _nh);

    public: ~KmeansGapClustering();

    public: void ExtractClusters(
	const std::vector<Eigen::Vector3f> &_vertices,
	Eigen::Vector2f _plane_center, Eigen::Vector2f _plane_size);

    public: std::vector<Eigen::Vector3f> GetClusterVertices(
        const std::vector<Eigen::Vector3f> &_vertices,
	const aero::box &_region);

    protected: void Subscribe(
        const std_msgs::Float32MultiArray::ConstPtr& _points);

    // append will reuse prior calculated initial_centers
    protected: void InitiateClusterCenterAppend(
	std::vector<point> &_points,
        std::vector<Eigen::Vector2f> &_initial_centers);

    protected: void InitiateClusterCenter(
	std::vector<point> &_points,
        std::vector<Eigen::Vector2f> &_initial_centers); // kmeans++

    protected: float ClusterPoints(
	std::vector<point> &_points,
        std::vector<Eigen::Vector2f> &_initial_centers,
	bool _append);

    protected: inline float NearestCluster(
        point _point, const std::vector<Eigen::Vector2f> &_initial_centers,
        bool _return_distance)
      {
	int nearest_cluster_id;
	float min_distance = 10000000000.0;

	for (unsigned int i = 0; i < _initial_centers.size(); ++i)
	{
	  Eigen::Vector2f diff = _point.pos - _initial_centers[i];
	  float distance = diff.dot(diff); // sum of squares
	  if (distance < min_distance)
	  {
	    min_distance = distance;
	    nearest_cluster_id = i;
	  }
	}

	if (_return_distance) return min_distance;
	else return static_cast<float>(nearest_cluster_id);
      };

    private: std::vector<point> points_;

    private: std::vector<aero::box> cluster_list_;

    private: int num_points_in_field_;

    private: int max_clusters_;

    private: int num_of_reference_sets_;

    private: int dim1;

    private: int dim2;

    private: ros::Publisher cluster_publisher_;

    private: ros::Publisher points_publisher_;

    private: ros::Subscriber subscriber_;
    };

    typedef std::shared_ptr<KmeansGapClustering> KmeansGapClusteringPtr;

  }
}

#endif
