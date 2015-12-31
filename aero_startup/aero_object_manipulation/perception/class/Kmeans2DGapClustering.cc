#include "aero_object_manipulation/perception/class/Kmeans2DGapClustering.hh"

using namespace aero;
using namespace common;

//////////////////////////////////////////////////
KmeansGapClustering::KmeansGapClustering(ros::NodeHandle _nh) : Base(_nh)
{
  num_points_in_field_ = 0;

  max_clusters_ = 10;
  num_of_reference_sets_ = 10;

  dim1 = 0;
  dim2 = 2;

  cluster_publisher_ =
      nh_.advertise<std_msgs::Float32MultiArray>("/kmeans/clusters", 1000);
  subscriber_ = nh_.subscribe("/point_cloud/points", 1,
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
    const std::vector<Eigen::Vector3f> &_vertices,
    Eigen::Vector2f _plane_center, Eigen::Vector2f _plane_size)
{
  ROS_INFO("clusters : %d", cluster_list_.size());

  // Eigen::Quaternionf world_q(base_to_eye_.orientation.x,
  // 			     base_to_eye_.orientation.y,
  // 			     base_to_eye_.orientation.z,
  // 			     base_to_eye_.orientation.w);
  // Eigen::Vector3f world_p(base_to_eye_.position.x,
  // 			  base_to_eye_.position.y,
  // 			  base_to_eye_.position.z);

  std::vector<point> points;
  points.reserve(_vertices.size());

  // create points and bounding box
  Eigen::Vector2f max = {_plane_center.x() + _plane_size.x(),
			 _plane_center.y() + _plane_size.y()};
  Eigen::Vector2f min = {_plane_center.x() - _plane_size.x(),
			 _plane_center.y() - _plane_size.y()};
  for (unsigned int i = 0; i < _vertices.size(); ++i)
  {
    // Eigen::Vector3f p = world_q * _vertices[i] + world_p;
    // Eigen::Vector2f p2d(p[dim1], p[dim2]);
    Eigen::Vector2f p2d(_vertices[i][dim1], _vertices[i][dim2]);
    points.push_back({p2d, 0, _vertices[i]});
  }

  ROS_INFO("points : %d -> %d", num_points_in_field_, points.size());
  num_points_in_field_ = points.size();

  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<float> rand(0.0, 1.0);
  int k = 1;
  std::vector<Eigen::Vector2f> initial_centers(1);
  initial_centers[0] =
      points[static_cast<int>(rand(mt) * points.size())].pos;
  std::uniform_real_distribution<float> x_range(min[0], max[0]);
  std::uniform_real_distribution<float> y_range(min[1], max[1]);

  ROS_INFO("looking for %f %f ~ %f %f", min[0], min[1], max[0], max[1]);

  // calculate first gap (1 cluster)
  Eigen::Vector2f center(0.0, 0.0);
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
    std::vector<Eigen::Vector2f> set;
    set.reserve(points.size());
    Eigen::Vector2f center(0.0, 0.0);
    for (unsigned int j = 0; j < points.size(); ++j)
    {
      set.push_back({x_range(mt), y_range(mt)});
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

  // ROS_INFO("k = 1 : Wk is %f, Wkb is %f, gap_k is %f", Wk, Wkb, gap_k);

  // calculate gap for more than 1 clusters
  bool gap_is_statisfied = false;
  // auto process_time_start = aero::time::now();
  while (!gap_is_statisfied && k < max_clusters_)
  {
    // auto process_time =
    //     aero::time::ms(aero::time::now() - process_time_start);
    // if (process_time > 3000) return; // abort process longer than 3 sec

    points_.clear();
    points_.resize(points.size());
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
	set.push_back({{x_range(mt), y_range(mt)}, 0});
      std::vector<Eigen::Vector2f> null(k); // filled in with ClusterPoints
      Wkb_sets[i] = ClusterPoints(set, null, false);
      Wkb += Wkb_sets[i];
    }
    Wkb /= num_of_reference_sets_;
    for (unsigned int i = 0; i < num_of_reference_sets_; ++i)
      sk += std::pow(Wkb_sets[i] - Wkb, 2);
    sk = sqrt(sk * (1 + 1/num_of_reference_sets_) / num_of_reference_sets_);
    float gap_k_1 = Wkb - Wk;

    // ROS_INFO("k = %d : Wk is %f, Wkb is %f, sk is %f, gap_k is %f",
    // 	     k, Wk, Wkb, sk, gap_k_1);

    if (gap_k >= gap_k_1 - sk) gap_is_statisfied = true;
    else gap_k = gap_k_1;
  }

  --k; // the number of clusters is k - 1

  // calculate bounding box of each cluster
  std::vector<Eigen::Vector3f> center_of_cluster(k, {0.0, 0.0, 0.0});
  std::vector<float> points_in_cluster(k, 0.0);
  std::vector<Eigen::Vector3f> max_point_value(k, {-1000.0, -1000.0, -1000.0});
  std::vector<Eigen::Vector3f> min_point_value(k, {1000.0, 1000.0, 1000.0});
  for (unsigned int i = 0; i < points_.size(); ++i)
  {
    int id = points_[i].group;
    center_of_cluster[id] += points_[i].vertex;
    ++points_in_cluster[id];
    if (points_[i].vertex.x() < min_point_value[id].x())
        min_point_value[id][0] = points_[i].vertex.x();
    else if (points_[i].vertex.x() > max_point_value[id].x())
        max_point_value[id][0] = points_[i].vertex.x();
    if (points_[i].vertex.y() < min_point_value[id].y())
        min_point_value[id][1] = points_[i].vertex.y();
    else if (points_[i].vertex.y() > max_point_value[id].y())
        max_point_value[id][1] = points_[i].vertex.y();
    if (points_[i].vertex.z() < min_point_value[id].z())
        min_point_value[id][2] = points_[i].vertex.z();
    else if (points_[i].vertex.z() > max_point_value[id].z())
        max_point_value[id][2] = points_[i].vertex.z();
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
	points_in_cluster[i] < 10)
      noise_candidates.push_back(cluster_i);
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
}

//////////////////////////////////////////////////
void KmeansGapClustering::Subscribe(
    const std_msgs::Float32MultiArray::ConstPtr& _points)
{
  auto start = aero::time::now();

  if (_points->layout.dim[0].size != 4)
  {
    ROS_FATAL("kmeans subscribes array not compatible");
    return;
  }

  Eigen::Vector2f plane_center(_points->data[0], _points->data[1]);
  Eigen::Vector2f plane_size(sqrt(_points->data[2]),
                             sqrt(_points->data[3]));

  std::vector<Eigen::Vector3f> vertices(_points->layout.dim[1].size);
  for (unsigned int i = 0; i < vertices.size(); ++i)
  {
    int data_idx =
        i * _points->layout.dim[1].stride + _points->layout.dim[0].size;
    vertices[i] = {_points->data[data_idx],
		   _points->data[data_idx + 1],
		   _points->data[data_idx + 2]};
  }

  this->ExtractClusters(vertices, plane_center, plane_size);

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

  ROS_WARN("cluster time : %f", aero::time::ms(aero::time::now() - start));
}

//////////////////////////////////////////////////
std::vector<Eigen::Vector3f> KmeansGapClustering::GetClusterVertices(
    const std::vector<Eigen::Vector3f> &_vertices,
    const aero::box &_region)
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
    std::vector<Eigen::Vector2f> &_initial_centers)
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
    std::vector<Eigen::Vector2f> current_initial_centers;
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
    std::vector<Eigen::Vector2f> &_initial_centers)
{
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<float> rand(0.0, 1.0);

  float seed = 0.0;
  std::vector<float> distance_to_nearest_cluster(_points.size());
  std::vector<Eigen::Vector2f> current_initial_centers;
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
    std::vector<Eigen::Vector2f> &_initial_centers,
    bool _append)
{
  if (_append)
    this->InitiateClusterCenterAppend(_points, _initial_centers);
  else
    this->InitiateClusterCenter(_points, _initial_centers);

  int changes = _points.size();
  while (changes > _points.size() * 0.005) // until 99.5% of points are stable
  {
    std::vector<Eigen::Vector2f> new_centers(
        _initial_centers.size(), {0.0, 0.0});
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
  std::vector<Eigen::Vector2f> centers(
      _initial_centers.size(), {0.0, 0.0});
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
