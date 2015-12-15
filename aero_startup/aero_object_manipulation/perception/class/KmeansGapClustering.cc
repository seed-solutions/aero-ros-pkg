#include "aero_object_manipulation/perception/class/KmeansGapClustering.hh"

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
KmeansGapClustering::KmeansGapClustering(ros::NodeHandle _nh) : nh_(_nh)
{
  field_range_max_ = {-100000.0, -100000.0, -100000.0};
  field_range_min_ = {100000.0, 100000.0, 100000.0};
  num_points_in_field_ = 0;
  scene_narrowed_n_times_ = 0;

  max_clusters_ = 10;
  num_of_reference_sets_ = 10;

  cluster_publisher_ =
      nh_.advertise<std_msgs::Float32MultiArray>("/kmeans/clusters", 1000);
  subscriber_ = nh_.subscribe("/point_cloud/points", 100,
			      &KmeansGapClustering::Subscribe, this);
  points_publisher_ = nh_.advertise<std_msgs::Float32MultiArray>(
      "/point_cloud/objects", 1000);
}

//////////////////////////////////////////////////
KmeansGapClustering::~KmeansGapClustering()
{
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

  if (cluster_list_.size() > 0) // keep calculating if no clusters are found
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
void KmeansGapClustering::Subscribe(
    const std_msgs::Float32MultiArray::ConstPtr& _points)
{
  std::vector<Eigen::Vector3f> vertices(_points->layout.dim[1].size);
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    int data_idx = i * _points->layout.dim[1].stride;
    vertices[i] = {_points->data[data_idx],
		   _points->data[data_idx + 1],
		   _points->data[data_idx + 2]};
  }

  this->ExtractClusters(vertices);

  std_msgs::Float32MultiArray c_msg;
  std_msgs::MultiArrayLayout layout;
  std_msgs::MultiArrayDimension dim_head;
  dim_head.label = "info";
  dim_head.size = 0;
  dim_head.stride = 1;
  layout.dim.push_back(dim_head);
  std_msgs::MultiArrayDimension dim;
  dim.label = "block1";
  dim.size = cluster_list_.size();
  dim.stride = 10; // center-xyz, max_bound-xyz, min_bound-xyz, points
  layout.dim.push_back(dim);
  c_msg.layout = layout;
  c_msg.data.reserve(dim.stride * dim.size);
  std::vector<std::vector<Eigen::Vector3f> > objects;
  objects.reserve(cluster_list_.size());
  for (unsigned int i = 0; i < cluster_list_.size(); ++i)
  {
    c_msg.data.push_back(cluster_list_[i].center.x);
    c_msg.data.push_back(cluster_list_[i].center.y);
    c_msg.data.push_back(cluster_list_[i].center.z);
    c_msg.data.push_back(cluster_list_[i].max_bound.x);
    c_msg.data.push_back(cluster_list_[i].max_bound.y);
    c_msg.data.push_back(cluster_list_[i].max_bound.z);
    c_msg.data.push_back(cluster_list_[i].min_bound.x);
    c_msg.data.push_back(cluster_list_[i].min_bound.y);
    c_msg.data.push_back(cluster_list_[i].min_bound.z);
    c_msg.data.push_back(cluster_list_[i].points);
    objects.push_back(this->GetClusterVertices(vertices, cluster_list_[i]));
  }
  cluster_publisher_.publish(c_msg);

  std_msgs::Float32MultiArray p_msg;
  layout.dim.clear();
  layout.dim.reserve(objects.size() + 1);
  dim_head.label = "info";
  dim_head.size = 0;
  dim_head.stride = objects.size();
  layout.dim.push_back(dim_head);
  for (unsigned int i = 0; i < objects.size(); ++i)
  {
    std_msgs::MultiArrayDimension dim;
    dim.label = "block" + std::to_string(i);
    dim.size = objects[i].size();
    dim.stride = 3;
    layout.dim.push_back(dim);
    int start_idx = p_msg.data.size();
    p_msg.data.resize(p_msg.data.size() + dim.size * dim.stride);
    for (unsigned int j = 0; j < objects[i].size(); ++j)
    {
      int point_idx = j * dim.stride;
      p_msg.data[start_idx + point_idx] = objects[i][j].x();
      p_msg.data[start_idx + point_idx + 1] = objects[i][j].y();
      p_msg.data[start_idx + point_idx + 2] = objects[i][j].z();
    }
  }
  p_msg.layout = layout;
  points_publisher_.publish(p_msg);
}

//////////////////////////////////////////////////
std::vector<Eigen::Vector3f> KmeansGapClustering::GetClusterVertices(
    std::vector<Eigen::Vector3f> _vertices, aero::box _region)
{
  std::vector<Eigen::Vector3f> object;
  object.reserve(_vertices.size());

  for (unsigned int i = 0; i < _vertices.size(); ++i)
    if (_vertices[i].x() < _region.max_bound.x &&
	_vertices[i].x() > _region.min_bound.x &&
	_vertices[i].y() < _region.max_bound.y &&
	_vertices[i].y() > _region.min_bound.y &&
	_vertices[i].z() < _region.max_bound.z &&
	_vertices[i].z() > _region.min_bound.z)
      object.push_back(_vertices[i]);

  object.resize(object.size());
  return object;
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
