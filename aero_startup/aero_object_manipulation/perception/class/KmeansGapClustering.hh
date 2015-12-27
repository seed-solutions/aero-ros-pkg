#ifndef _AERO_COMMON_KMEANS_GAP_CLUSTERING_H_
#define _AERO_COMMON_KMEANS_GAP_CLUSTERING_H_

#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <Eigen/Core>
#include "aero_common/types.h"
#include "aero_common/time.h"

namespace aero
{
  namespace common
  {

    class KmeansGapClustering
    {
    private: struct point { Eigen::Vector3f pos; int group; };

    public: KmeansGapClustering();

    public: explicit KmeansGapClustering(ros::NodeHandle _nh);

    public: ~KmeansGapClustering();

    public: void ExtractClusters(std::vector<Eigen::Vector3f> _vertices);

    public: std::vector<Eigen::Vector3f> GetClusterVertices(
        std::vector<Eigen::Vector3f> _vertices, aero::box _region);

    protected: void Subscribe(
        const std_msgs::Float32MultiArray::ConstPtr& _points);

    // append will reuse prior calculated initial_centers
    protected: void InitiateClusterCenterAppend(
	std::vector<point> &_points,
        std::vector<Eigen::Vector3f> &_initial_centers);

    protected: void InitiateClusterCenter(
	std::vector<point> &_points,
        std::vector<Eigen::Vector3f> &_initial_centers); // kmeans++

    protected: float ClusterPoints(
	std::vector<point> &_points,
        std::vector<Eigen::Vector3f> &_initial_centers,
	bool _append);

    protected: inline float NearestCluster(
        point _point, std::vector<Eigen::Vector3f> _initial_centers,
        bool _return_distance)
      {
	int nearest_cluster_id;
	float min_distance = 10000000000.0;

	for (unsigned int i = 0; i < _initial_centers.size(); ++i)
	{
	  Eigen::Vector3f diff = _point.pos - _initial_centers[i];
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

    protected: inline bool IsNoise(Eigen::Vector3f _p)
      {
	for (unsigned int i = 0; i < noises_.size(); ++i)
	  if (_p.x() < noises_[i].max_bound.x &&
	      _p.x() > noises_[i].min_bound.x &&
	      _p.y() < noises_[i].max_bound.y &&
	      _p.y() > noises_[i].min_bound.y &&
	      _p.z() < noises_[i].max_bound.z &&
	      _p.z() > noises_[i].min_bound.z)
	    return true;

	return false;
      };

    private: std::vector<point> points_;

    private: std::vector<aero::box> cluster_list_;

    private: std::vector<aero::box> noises_;

    private: aero::xyz field_range_max_;

    private: aero::xyz field_range_min_;

    private: int num_points_in_field_;

    private: int scene_narrowed_n_times_;

    private: int max_clusters_;

    private: int num_of_reference_sets_;

    private: ros::NodeHandle nh_;

    private: ros::Publisher cluster_publisher_;

    private: ros::Publisher points_publisher_;

    private: ros::Subscriber subscriber_;
    };

    typedef std::shared_ptr<KmeansGapClustering> KmeansGapClusteringPtr;

  }
}

#endif
