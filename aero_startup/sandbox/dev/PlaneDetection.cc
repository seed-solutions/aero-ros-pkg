#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf/transform_broadcaster.h>
#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_common/types.h"

#include <random>

namespace aero
{
  namespace common
  {

    class KmeansGapClustering
    {
    private: struct point { Eigen::Vector3f pos; int group; };

    public: KmeansGapClustering();

    public: ~KmeansGapClustering();

    public: void ExtractClusters(std::vector<Eigen::Vector3f> _vertices);

    public: aero::box GetLargestCluster();

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

    protected: float NearestCluster( // inline
        point _point, std::vector<Eigen::Vector3f> _initial_centers,
        bool _return_distance);

    protected: bool IsNoise(Eigen::Vector3f _p); // inline

    private: std::vector<point> points_;

    private: std::vector<aero::box> cluster_list_;

    private: std::vector<aero::box> noises_;

    private: aero::xyz field_range_max_;

    private: aero::xyz field_range_min_;

    private: int num_points_in_field_;

    private: int scene_narrowed_n_times_;

    private: int max_clusters_;

    private: int num_of_reference_sets_;
    };

    typedef std::shared_ptr<KmeansGapClustering> KmeansGapClusteringPtr;

  }
}

namespace aero
{
  namespace perception
  {

    class PlaneDetectedPointCloud : public PointCloudSensor
    {
    public: explicit PlaneDetectedPointCloud(ros::NodeHandle _nh);

    public: ~PlaneDetectedPointCloud();

    protected: void SubscribePoints(
	const sensor_msgs::PointCloud2::ConstPtr& _msg);

    public: std::vector<Eigen::Vector3f> GetObjectVertices(aero::box _region);

    protected: ros::Subscriber point_cloud_listener_;

    protected: Eigen::Vector3f desk_plane_norm_;

    protected: float height_range_per_region_;

    protected: ros::Publisher pcl_pub_;

    protected: aero::common::KmeansGapClusteringPtr kmeans_;
    };

    typedef std::shared_ptr<PlaneDetectedPointCloud> PlaneDetectedPointCloudPtr;

  }
}

using namespace aero;
using namespace common;

//////////////////////////////////////////////////
KmeansGapClustering::KmeansGapClustering()
{
  field_range_max_ = {-100000.0, -100000.0, -100000.0};
  field_range_min_ = {100000.0, 100000.0, 100000.0};
  num_points_in_field_ = 0;
  scene_narrowed_n_times_ = 0;

  max_clusters_ = 10;
  num_of_reference_sets_ = 10;
}

//////////////////////////////////////////////////
KmeansGapClustering::~KmeansGapClustering()
{
}

//////////////////////////////////////////////////
bool KmeansGapClustering::IsNoise(Eigen::Vector3f _p) // inline
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
}

//////////////////////////////////////////////////
void KmeansGapClustering::ExtractClusters(
    std::vector<Eigen::Vector3f> _vertices)
{
  ROS_INFO("clusters : %d, noises : %d", cluster_list_.size(), noises_.size());

  std::vector<point> points;
  points.reserve(_vertices.size());

  // create points and bounding box
  Eigen::Vector3f max = {-100000.0, -100000.0, -100000.0};
  Eigen::Vector3f min = {100000.0, 100000.0, 100000.0};
  for (unsigned int i = 0; i < _vertices.size(); ++i)
  {
    // reject noise points
    if (this->IsNoise(_vertices[i])) continue;
    points.push_back({_vertices[i], 0});
    if (_vertices[i].x() < min.x()) min[0] = _vertices[i].x();
    else if (_vertices[i].x() > max.x()) max[0] = _vertices[i].x();
    if (_vertices[i].y() < min.y()) min[1] = _vertices[i].y();
    else if (_vertices[i].y() > max.y()) max[1] = _vertices[i].y();
    if (_vertices[i].z() < min.z()) min[2] = _vertices[i].z();
    else if (_vertices[i].z() > max.z()) max[2] = _vertices[i].z();
  }
  points.resize(points.size());


  ROS_INFO("field_max : %f %f %f -> %f %f %f",
	   field_range_max_.x, field_range_max_.y, field_range_max_.z,
	   max.x(), max.y(), max.z());
  if (fabs(max.x() - field_range_max_.x) >= 0.2)
    ROS_WARN("qualifies max.x");
  if (fabs(max.y() - field_range_max_.y) >= 0.1)
    ROS_WARN("qualifies max.y");
  if (fabs(max.z() - field_range_max_.z) >= 0.1)
    ROS_WARN("qualifies max.z");

  ROS_INFO("field_min : %f %f %f -> %f %f %f",
	   field_range_min_.x, field_range_min_.y, field_range_min_.z,
	   min.x(), min.y(), min.z());
  if (fabs(min.x() - field_range_min_.x) >= 0.2)
    ROS_WARN("qualifies min.x");
  if (fabs(min.y() - field_range_min_.y) >= 0.1)
    ROS_WARN("qualifies min.x");
  if (fabs(min.z() - field_range_min_.z) >= 0.1)
    ROS_WARN("qualifies min.x");

  ROS_INFO("points : %d -> %d", num_points_in_field_, points.size());
  if (abs(static_cast<int>(points.size() - num_points_in_field_)) >= 300)
    ROS_WARN("qualifies points");

  // if not much of a change in scene, don't calculate
  if (fabs(max.x() - field_range_max_.x) < 0.2 && // ps4eye is bad at x
      fabs(min.x() - field_range_min_.x) < 0.2 && // ps4eye is bad at x
      fabs(max.y() - field_range_max_.y) < 0.1 &&
      fabs(min.y() - field_range_min_.y) < 0.1 &&
      fabs(max.z() - field_range_max_.z) < 0.1 &&
      fabs(min.z() - field_range_min_.z) < 0.1 &&
      abs(static_cast<int>(points.size() - num_points_in_field_)) < 300)
    return;

  // if scene was enlarged by more than 15cm, clear noises
  // when the scene was shrinken, that could mean noises were eliminated
  // in that case, we want to keep the current noise information
  if ((max.x() - field_range_max_.x) > 0.3 || // ps4eye is bad at x
      (min.x() - field_range_min_.x) < -0.3 || // ps4eye is bad at x
      (max.y() - field_range_max_.y) > 0.15 ||
      (min.y() - field_range_min_.y) < -0.15 ||
      (max.z() - field_range_max_.z) > 0.15 ||
      (min.z() - field_range_min_.z) < -0.15)
  {
    if (static_cast<int>(points.size() - num_points_in_field_) > 500)
      noises_.clear();
  }
  else // spacial shrink case
  {
    ++scene_narrowed_n_times_;
    // information might have been lost for a moment
    if (scene_narrowed_n_times_ < 5) return;
    scene_narrowed_n_times_ = 0;
  }

  field_range_max_ = {max.x(), max.y(), max.z()};
  field_range_min_ = {min.x(), min.y(), min.z()};
  num_points_in_field_ = points.size();

  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<float> rand(0.0, 1.0);
  int k = 1;
  std::vector<Eigen::Vector3f> initial_centers(1);
  initial_centers[0] =
      points[static_cast<int>(rand(mt) * points.size())].pos;
  std::uniform_real_distribution<float> x_range(min[0], max[0]);
  std::uniform_real_distribution<float> y_range(min[1], max[1]);
  std::uniform_real_distribution<float> z_range(min[2], max[2]);

  // calculate first gap (1 cluster)
  Eigen::Vector3f center = {0.0, 0.0, 0.0};
  for (unsigned int i = 0; i < points.size(); ++i)
    center += points[i].pos;
  center /= points.size();
  float Wk = 0.0;
  for (unsigned int i = 0; i < points.size(); ++i)
    Wk += std::pow((points[i].pos - center).norm(), 2);
  Wk = log(Wk);
  // create monte carlo reference sets
  float Wkb = 0.0;
  for (unsigned int i = 0; i < num_of_reference_sets_; ++i)
  {
    std::vector<Eigen::Vector3f> set;
    set.reserve(points.size());
    Eigen::Vector3f center = {0.0, 0.0, 0.0};
    for (unsigned int j = 0; j < points.size(); ++j)
    {
      set.push_back({x_range(mt), y_range(mt), z_range(mt)});
      center += set[j];
    }
    center /= set.size();
    float Wkb_i = 0.0;
    for (unsigned int j = 0; j < set.size(); ++j)
      Wkb_i += std::pow((set[j] - center).norm(), 2);
    Wkb += log(Wkb_i);
  }
  Wkb /= num_of_reference_sets_;
  float gap_k = Wkb - Wk;

  ROS_INFO("k = 1 : Wk is %f, Wkb is %f, gap_k is %f", Wk, Wkb, gap_k);

  // calculate gap for more than 1 clusters
  bool gap_is_statisfied = false;
  while (!gap_is_statisfied && k < max_clusters_)
  {
    points_.clear();
    points_.resize(_vertices.size());
    points_.assign(points.begin(), points.end());

    ++k;
    initial_centers.resize(k); // expand number of clusters to k clusters
    // segmentate points with k-means
    float Wk = ClusterPoints(points, initial_centers, true);

    // create monte carlo reference sets
    float Wkb = 0.0;
    float sk = 0.0;
    std::vector<float> Wkb_sets(num_of_reference_sets_, 0.0);
    for (unsigned int i = 0; i < num_of_reference_sets_; ++i)
    {
      std::vector<point> set;
      set.reserve(points.size());
      for (unsigned int j = 0; j < points.size(); ++j)
	set.push_back({{x_range(mt), y_range(mt), z_range(mt)}, 0});
      set.resize(points.size());
      std::vector<Eigen::Vector3f> null(k); // filled in with ClusterPoints
      Wkb_sets[i] = ClusterPoints(set, null, false);
      Wkb += Wkb_sets[i];
    }
    Wkb /= num_of_reference_sets_;
    for (unsigned int i = 0; i < num_of_reference_sets_; ++i)
      sk += std::pow(Wkb_sets[i] - Wkb, 2);
    sk = sqrt(sk * (1 + 1/num_of_reference_sets_) / num_of_reference_sets_);
    float gap_k_1 = Wkb - Wk;

    ROS_INFO("k = %d : Wk is %f, Wkb is %f, sk is %f, gap_k is %f",
	     k, Wk, Wkb, sk, gap_k_1);

    if (gap_k >= gap_k_1 - sk) gap_is_statisfied = true;
    else gap_k = gap_k_1;
  }

  --k; // the number of clusters is k - 1

  // calculate bounding box of each cluster
  std::vector<Eigen::Vector3f> center_of_cluster(k, {0.0, 0.0, 0.0});
  std::vector<float> points_in_cluster(k, 0.0);
  std::vector<Eigen::Vector3f> max_point_value(k, min);
  std::vector<Eigen::Vector3f> min_point_value(k, max);
  for (unsigned int i = 0; i < points_.size(); ++i)
  {
    int id = points_[i].group;
    center_of_cluster[id] += points_[i].pos;
    ++points_in_cluster[id];
    if (points_[i].pos.x() < min_point_value[id].x())
        min_point_value[id][0] = points_[i].pos.x();
    else if (points_[i].pos.x() > max_point_value[id].x())
        max_point_value[id][0] = points_[i].pos.x();
    if (points_[i].pos.y() < min_point_value[id].y())
        min_point_value[id][1] = points_[i].pos.y();
    else if (points_[i].pos.y() > max_point_value[id].y())
        max_point_value[id][1] = points_[i].pos.y();
    if (points_[i].pos.z() < min_point_value[id].z())
        min_point_value[id][2] = points_[i].pos.z();
    else if (points_[i].pos.z() > max_point_value[id].z())
        max_point_value[id][2] = points_[i].pos.z();
  }

  // add bounding box data to cluster_list_ or noises_
  std::vector<aero::box> cluster_candidates;
  std::vector<aero::box> noise_candidates;
  cluster_candidates.reserve(k);
  noise_candidates.reserve(k);
  for (unsigned int i = 0; i < k; ++i)
  {
    Eigen::Vector3f center = center_of_cluster[i] / points_in_cluster[i];

    aero::box cluster_i;
    cluster_i.center = {center.x(), center.y(), center.z()};
    cluster_i.max_bound =
      {max_point_value[i].x(), max_point_value[i].y(), max_point_value[i].z()};
    cluster_i.min_bound =
      {min_point_value[i].x(), min_point_value[i].y(), min_point_value[i].z()};
    cluster_i.points = points_in_cluster[i];

    if ((max_point_value[i] - min_point_value[i]).norm() < 0.05 ||
	points_in_cluster[i] < 500)
      noise_candidates.push_back(cluster_i); // we can always add new noises
    else
      cluster_candidates.push_back(cluster_i);
  }
  cluster_candidates.resize(cluster_candidates.size());
  noise_candidates.resize(noise_candidates.size());
  if (noise_candidates.size() > cluster_candidates.size()) // bad clustering
    return;

  cluster_list_.clear();
  cluster_list_.reserve(k);
  cluster_list_.assign(cluster_candidates.begin(), cluster_candidates.end());
  for (unsigned int i = 0; i < noise_candidates.size(); ++i)
    noises_.push_back(noise_candidates[i]);
}

//////////////////////////////////////////////////
aero::box KmeansGapClustering::GetLargestCluster()
{
  int max_id = 0;
  int largest_cluster = 0;
  for (unsigned int i = 0; i < cluster_list_.size(); ++i)
    if (cluster_list_[i].points > largest_cluster)
    {
      max_id = i;
      largest_cluster = cluster_list_[i].points;
    }

  if (cluster_list_.size() == 0)
    return {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0};

  return cluster_list_[max_id];
}

//////////////////////////////////////////////////
float KmeansGapClustering::NearestCluster( // inline
    point _p, std::vector<Eigen::Vector3f> _initial_centers,
    bool _return_distance)
{
  int nearest_cluster_id;
  float min_distance = 10000000000.0;

  for (unsigned int i = 0; i < _initial_centers.size(); ++i)
  {
    Eigen::Vector3f diff = _p.pos - _initial_centers[i];
    float distance = diff.dot(diff); // sum of squares
    if (distance < min_distance)
    {
      min_distance = distance;
      nearest_cluster_id = i;
    }
  }

  if (_return_distance) return min_distance;
  else return static_cast<float>(nearest_cluster_id);
}

//////////////////////////////////////////////////
void KmeansGapClustering::InitiateClusterCenter(
    std::vector<point> &_points,
    std::vector<Eigen::Vector3f> &_initial_centers)
{
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<float> rand(0.0, 1.0);

  _initial_centers[0] =
      _points[static_cast<int>(rand(mt) * _points.size())].pos;

  for (unsigned int i = 1; i < _initial_centers.size(); ++i)
  {
    float seed = 0.0;
    std::vector<float> distance_to_nearest_cluster(_points.size());
    std::vector<Eigen::Vector3f> current_initial_centers;
    current_initial_centers.assign(_initial_centers.begin(),
				   _initial_centers.begin() + i);

    // get the distances to nearest cluster for each point
    for (unsigned int j = 0; j < _points.size(); ++j)
    {
      distance_to_nearest_cluster[j] =
          this->NearestCluster(_points[j], current_initial_centers, true);
      seed += distance_to_nearest_cluster[j];
    }

    // set a random center from sum of all distances
    seed *= rand(mt);
    for (unsigned int j = 0; _points.size(); ++j)
    {
      seed -= distance_to_nearest_cluster[j];
      if (seed > 0) continue;
      _initial_centers[i] = _points[j].pos;
      break;
    }
  }

  for (unsigned int j = 0; j < _points.size(); ++j)
    _points[j].group =
        this->NearestCluster(_points[j], _initial_centers, false);
}

//////////////////////////////////////////////////
void KmeansGapClustering::InitiateClusterCenterAppend(
    std::vector<point> &_points,
    std::vector<Eigen::Vector3f> &_initial_centers)
{
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<float> rand(0.0, 1.0);

  float seed = 0.0;
  std::vector<float> distance_to_nearest_cluster(_points.size());
  std::vector<Eigen::Vector3f> current_initial_centers;
  current_initial_centers.assign(_initial_centers.begin(),
				 _initial_centers.end() - 1);

  // get the distances to nearest cluster for each point
  for (unsigned int j = 0; j < _points.size(); ++j)
  {
    distance_to_nearest_cluster[j] =
        this->NearestCluster(_points[j], current_initial_centers, true);
    seed += distance_to_nearest_cluster[j];
  }

  // set a random center from sum of all distances
  seed *= rand(mt);
  for (unsigned int j = 0; _points.size(); ++j)
  {
    seed -= distance_to_nearest_cluster[j];
    if (seed > 0) continue;
    _initial_centers[_initial_centers.size() - 1] = _points[j].pos;
    break;
  }

  for (unsigned int j = 0; j < _points.size(); ++j)
    _points[j].group =
        this->NearestCluster(_points[j], _initial_centers, false);
}

//////////////////////////////////////////////////
float KmeansGapClustering::ClusterPoints(
    std::vector<point> &_points,
    std::vector<Eigen::Vector3f> &_initial_centers,
    bool _append)
{
  if (_append)
    this->InitiateClusterCenterAppend(_points, _initial_centers);
  else
    this->InitiateClusterCenter(_points, _initial_centers);

  int changes = _points.size();
  while (changes > _points.size() * 0.005) // until 99.5% of points are stable
  {
    std::vector<Eigen::Vector3f> new_centers(
        _initial_centers.size(), {0.0, 0.0, 0.0});
    std::vector<int> num_of_points(new_centers.size(), 0);

    for (unsigned int i = 0; i < _points.size(); ++i)
    {
      new_centers[_points[i].group] += _points[i].pos;
      ++num_of_points[_points[i].group];
    }

    for (unsigned int i = 0; i < new_centers.size(); ++i)
      new_centers[i] /= num_of_points[i];

    changes = 0;
    for (unsigned int i = 0; i < _points.size(); ++i)
    {
      int nearest_cluster_id =
	  this->NearestCluster(_points[i], new_centers, false);
      if (nearest_cluster_id == _points[i].group) continue;
      ++changes;
      _points[i].group = nearest_cluster_id;
    }

    // ROS_INFO("%d out of %d are unstable", changes, _points.size());
  }

  // calculate center of each cluster
  std::vector<Eigen::Vector3f> centers(
      _initial_centers.size(), {0.0, 0.0, 0.0});
  std::vector<int> num_of_points(centers.size(), 0);

  for (unsigned int i = 0; i < _points.size(); ++i)
  {
    centers[_points[i].group] += _points[i].pos;
    ++num_of_points[_points[i].group];
  }
  for (unsigned int i = 0; i < centers.size(); ++i)
    centers[i] /= num_of_points[i];

  // calculate Wk
  float Wk = 0.0;
  for (unsigned int i = 0; i < _points.size(); ++i)
    Wk += std::pow((_points[i].pos - centers[_points[i].group]).norm(), 2);

  return log(Wk);
}

using namespace perception;

//////////////////////////////////////////////////
PlaneDetectedPointCloud::PlaneDetectedPointCloud(ros::NodeHandle _nh)
    : PointCloudSensor(_nh)
{
  height_range_per_region_ = 0.03;

  desk_plane_norm_ =
      Eigen::Quaternionf(1, 0, 0, 0) * Eigen::Vector3f(0, 0, 1);

  space_min_ = {-0.5, -0.5, 0.8};
  space_max_ = {0.5, 1.0, 5.0};

  point_cloud_listener_ =
      nh_.subscribe("/stereo/points2", 1000,
		    &PlaneDetectedPointCloud::SubscribePoints, this);

  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/visualized_object_pcl", 100);

  kmeans_.reset(new aero::common::KmeansGapClustering());
}

//////////////////////////////////////////////////
PlaneDetectedPointCloud::~PlaneDetectedPointCloud()
{
}

//////////////////////////////////////////////////
std::vector<Eigen::Vector3f> PlaneDetectedPointCloud::GetObjectVertices(
    aero::box _region)
{
  std::vector<Eigen::Vector3f> object;
  object.reserve(vertices_.size());

  for (unsigned int i = 0; i < vertices_.size(); ++i)
    if (vertices_[i].x() < _region.max_bound.x &&
	vertices_[i].x() > _region.min_bound.x &&
	vertices_[i].y() < _region.max_bound.y &&
	vertices_[i].y() > _region.min_bound.y &&
	vertices_[i].z() < _region.max_bound.z &&
	vertices_[i].z() > _region.min_bound.z)
      object.push_back(vertices_[i]);

  object.resize(object.size());
  return object;
}

//////////////////////////////////////////////////
void PlaneDetectedPointCloud::SubscribePoints(
    const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  Eigen::Vector3f manipulation_space_max(
      space_max_.x, space_max_.y, space_max_.z);
  Eigen::Vector3f manipulation_space_min(
      space_min_.x, space_min_.y, space_min_.z);

  float height_range_total =
    (manipulation_space_max - manipulation_space_min).norm();
  int num_of_regions =
    height_range_total / height_range_per_region_ + 1;
  float lowest_height_level = -height_range_total;

  static tf::TransformListener tl;
  tf::StampedTransform base_to_eye;
  Eigen::Quaternionf base_to_eye_q;
  ros::Time now = ros::Time::now();
  tl.waitForTransform("leg_base_link", "ps4eye_frame",
		      now, ros::Duration(2.0));

  try
  {
    tl.lookupTransform("leg_base_link", "ps4eye_frame", now, base_to_eye);
  }
  catch (std::exception e)
  {
    ROS_ERROR("failed tf listen");
    return;
  }

  base_to_eye_q =
    Eigen::Quaternionf(base_to_eye.getRotation().w(),
		       base_to_eye.getRotation().x(),
		       base_to_eye.getRotation().y(),
		       base_to_eye.getRotation().z());
  desk_plane_norm_ =
    base_to_eye_q.inverse() * Eigen::Vector3f(0, 0, -1);

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl, *raw);

  std::vector<std::vector<Eigen::Vector3f> > vertices_block(num_of_regions);
  std::vector<std::vector<Eigen::Vector3f> > vertices_shifted(num_of_regions);
  std::vector<int> elements_in_block(num_of_regions, 0);

  for (unsigned int i = 0; i < raw->points.size(); ++i)
    if ((raw->points[i].x > manipulation_space_min.x()) &&
	(raw->points[i].x < manipulation_space_max.x()) &&
	(raw->points[i].y > manipulation_space_min.y()) &&
	(raw->points[i].y < manipulation_space_max.y()) &&
	(raw->points[i].z > manipulation_space_min.z()) &&
	(raw->points[i].z < manipulation_space_max.z()))
    {
      Eigen::Vector3f point(
	  raw->points[i].x, raw->points[i].y, raw->points[i].z);
      int index =
          static_cast<int>(
	      ((manipulation_space_min - point).dot(desk_plane_norm_) -
	       lowest_height_level) / height_range_per_region_);
      int index_shifted =
          static_cast<int>(
	      ((manipulation_space_min - point).dot(desk_plane_norm_) -
	       lowest_height_level) / height_range_per_region_ - 0.5);
      if (index_shifted < 0) index_shifted = 0;
      Eigen::Vector3f tmp = manipulation_space_min - point;
      vertices_block[index].push_back(point);
      vertices_shifted[index_shifted].push_back(point);
    }

  // find table (continuous non-zero area)

  int start_from = 0;
  for (unsigned int i = 0; i < num_of_regions; ++i)
  {
    if (vertices_block[i].size() == 0) continue;

    int region_begin = i;
    while (vertices_block[i].size() > 0 && i < num_of_regions)
      ++i;
    if ((i - region_begin) * height_range_per_region_ > 0.5) // 50 cm
      // reject floor
      start_from = region_begin + (i - region_begin) * 0.5;
  }

  // find area with most points in highest plane (eliminate floor)

  int max_index = -1;
  int max_points = 0;
  int max_index_when_shifted = -1;
  int max_points_when_shifted = 0;
  // for (int i = plane_idx[plane_idx.size()-1]; i < num_of_regions; ++i)
  for (unsigned int i = start_from; i < num_of_regions; ++i)
  {
    if (vertices_block[i].size() == 0) continue;

    ROS_WARN("block %d has %d points", i, vertices_block[i].size());
    if (vertices_block[i].size() > max_points)
    {
      max_points = vertices_block[i].size();
      max_index = i;
    }
    if (vertices_shifted[i].size() > max_points_when_shifted)
    {
      max_points_when_shifted = vertices_shifted[i].size();
      max_index_when_shifted = i;
    }
  }

  // get the better region cut

  std::vector<Eigen::Vector3f> plane_points;
  if (max_points_when_shifted > max_points)
    plane_points.assign(vertices_shifted[max_index_when_shifted].begin(),
			vertices_shifted[max_index_when_shifted].end());
  else
    plane_points.assign(vertices_block[max_index].begin(),
			vertices_block[max_index].end());

  // get the size of plane

  // cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // cloud_->points.reserve(plane_points.size());

  Eigen::Vector3f plane_center(0, 0, 0);
  for (unsigned int j = 0; j < plane_points.size(); ++j)
  {
    plane_center += plane_points[j];
    // cloud_->points.push_back(pcl::PointXYZ(plane_points[j].x(),
    // 					   plane_points[j].y(),
    // 					   plane_points[j].z()));
  }
  plane_center = plane_center * (1.0 / plane_points.size());

  Eigen::Vector3f plane_variance(0, 0, 0);
  for (unsigned int j = 0; j < plane_points.size(); ++j)
  {
    Eigen::Vector3f variance = Eigen::Vector3f(
	(plane_points[j].x() - plane_center.x()) *
            (plane_points[j].x() - plane_center.x()),
        (plane_points[j].y() - plane_center.y()) *
            (plane_points[j].y() - plane_center.y()),
        (plane_points[j].z() - plane_center.z()) *
            (plane_points[j].z() - plane_center.z()));
    plane_variance += variance;
  }
  plane_variance = plane_variance * (4.0 / plane_points.size()); // cut 5%

  // get the points on plane

  vertices_.clear();
  vertices_.reserve(raw->points.size());
  if (max_points_when_shifted > max_points)
    for (unsigned int i = // objects higher than 5 cm
	   (max_index_when_shifted + 1 + 0.05 / height_range_per_region_);
	 i < vertices_shifted.size(); ++i)
      for (unsigned int j = 0; j < vertices_shifted[i].size(); ++j)
      {
	if ((vertices_shifted[i][j].x() - plane_center.x()) *
	      (vertices_shifted[i][j].x() - plane_center.x()) <
	    plane_variance.x() &&
	    // (vertices_shifted[i][j].y() - plane_center.y()) *
	    //   (vertices_shifted[i][j].y() - plane_center.y()) <
	    // plane_variance.y() &&
	    (vertices_shifted[i][j].z() - plane_center.z()) *
	      (vertices_shifted[i][j].z() - plane_center.z()) <
	    plane_variance.z())
	  vertices_.push_back(vertices_shifted[i][j]);
      }
  else
    for (unsigned int i = // objects higher than 5 cm
	   (max_index + 1 + 0.05 / height_range_per_region_);
	 i < vertices_block.size(); ++i)
      for (unsigned int j = 0; j < vertices_block[i].size(); ++j)
      {
	if ((vertices_block[i][j].x() - plane_center.x()) *
	      (vertices_block[i][j].x() - plane_center.x()) <
	    plane_variance.x() &&
	    // (vertices_block[i][j].y() - plane_center.y()) *
	    //   (vertices_block[i][j].y() - plane_center.y()) <
	    // plane_variance.y() &&
	    (vertices_block[i][j].z() - plane_center.z()) *
	      (vertices_block[i][j].z() - plane_center.z()) <
	    plane_variance.z())
	  vertices_.push_back(vertices_block[i][j]);
      }
  vertices_.resize(vertices_.size());

  // kmeans++ gap clustrering

  kmeans_->ExtractClusters(vertices_);
  GetObjectVertices(kmeans_->GetLargestCluster());

  // now we have the points on the plane

  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->points.reserve(plane_points.size());

  for (unsigned int i = 0; i < vertices_.size(); ++i)
  {
    cloud_->points.push_back(pcl::PointXYZ(vertices_[i].x(),
					   vertices_[i].y(),
					   vertices_[i].z()));
  }

  pcl::PCLPointCloud2 pcl_out;
  sensor_msgs::PointCloud2 msg;
  pcl::toPCLPointCloud2(*cloud_, pcl_out);
  pcl_conversions::fromPCL(pcl_out, msg);
  msg.header.frame_id = "ps4eye_frame";
  msg.header.stamp = ros::Time(0); // get possible recent
  pcl_pub_.publish(msg);
}


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "segmentation");
  ros::NodeHandle nh;

  aero::perception::PlaneDetectedPointCloudPtr sensor(
      new aero::perception::PlaneDetectedPointCloud(nh));

  ros::spin();

  return 0;
}
