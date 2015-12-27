#include "aero_object_manipulation/perception/class/ClusteredCloud.hh"

using namespace aero;
using namespace common;

//////////////////////////////////////////////////
ClusteredCloud::ClusteredCloud(ros::NodeHandle _nh) : nh_(_nh)
{
  point_publisher_ =
      nh_.advertise<std_msgs::Float32MultiArray>("/cluster_cloud/points", 1000);

  points_subscriber_ = nh_.subscribe("/point_cloud/objects", 1000,
				     &ClusteredCloud::Subscribe, this);

  cluster_subscriber_ = nh_.subscribe("/kmeans/clusters", 1000,
				      &ClusteredCloud::SubscribeClusters, this);

  return_cluster_from_center_ =
    nh_.advertiseService("/cluster_cloud/get_object_from_center",
			 &ClusteredCloud::CallClusterFromCenter, this);

  process_sleep_ = false;
  this->publish_func_ = [=](){ this->PublishAllClusters(); };
  this->default_func_ = this->publish_func_;
}

//////////////////////////////////////////////////
ClusteredCloud::~ClusteredCloud()
{
}

//////////////////////////////////////////////////
void ClusteredCloud::Spin()
{
  this->publish_func_();
}

//////////////////////////////////////////////////
void ClusteredCloud::Subscribe(
    const std_msgs::Float32MultiArray::ConstPtr& _points)
{
  if (process_sleep_)
  {
    auto sleeped_time = aero::time::ms(aero::time::now() - sleep_start_);
    if (sleeped_time > 10000) process_sleep_ = false;
    else return; // this handles late arrived kmeans points after process kill
  }

  clouds_.clear();
  int num_of_clusters = _points->layout.dim[0].stride;
  clouds_.reserve(num_of_clusters);

  int data_idx = 0;
  for (unsigned int i = 1; i <= num_of_clusters; ++i)
  {
    int points_in_cluster = _points->layout.dim[i].size;
    std::vector<Eigen::Vector3f> points;
    points.reserve(points_in_cluster);
    for (unsigned int j = 0; j < points_in_cluster; ++j)
    {
      int point_idx = j * _points->layout.dim[i].stride;
      Eigen::Vector3f point = {_points->data[data_idx + point_idx],
			       _points->data[data_idx + point_idx + 1],
			       _points->data[data_idx + point_idx + 2]};
      points.push_back(point);
    }
    data_idx += points_in_cluster * _points->layout.dim[i].stride;
    clouds_.push_back(points);
  }

  this->publish_func_ = this->default_func_;
}

//////////////////////////////////////////////////
void ClusteredCloud::SubscribeClusters(
    const std_msgs::Float32MultiArray::ConstPtr& _clusters)
{
  if (process_sleep_) return;

  clusters_.clear();
  int num_of_clusters = _clusters->layout.dim[1].size;
  clusters_.reserve(num_of_clusters);

  for (unsigned int i = 0; i < num_of_clusters; ++i)
  {
    int data_idx = i * _clusters->layout.dim[1].stride;
    aero::xyz center = {_clusters->data[data_idx],
			_clusters->data[data_idx + 1],
			_clusters->data[data_idx + 2]};
    aero::xyz max_bound = {_clusters->data[data_idx + 3],
			   _clusters->data[data_idx + 4],
			   _clusters->data[data_idx + 5]};
    aero::xyz min_bound = {_clusters->data[data_idx + 6],
			   _clusters->data[data_idx + 7],
			   _clusters->data[data_idx + 8]};
    aero::box cluster =
      {center, max_bound, min_bound, _clusters->data[data_idx + 9]};
    clusters_.push_back(cluster);
  }
}

//////////////////////////////////////////////////
bool ClusteredCloud::CallClusterFromCenter(
    aero_startup::BoxFromXYZ::Request &_req,
    aero_startup::BoxFromXYZ::Response &_res)
{
  int nearest_cluster_id = 0;
  float nearest_distance = 10000000.0;
  for (unsigned int i = 0; i < clusters_.size(); ++i)
  {
    float distance =
      std::pow(clusters_[i].center.x - _req.x, 2) +
      std::pow(clusters_[i].center.y - _req.y, 2) +
      std::pow(clusters_[i].center.z - _req.z, 2);
    if (distance < nearest_distance)
    {
      nearest_cluster_id = i;
      nearest_distance = distance;
    }
  }

  if (clusters_.size() == 0)
    return false;

  if (_req.kill_spin)
  {
    this->publish_func_ = [=](){ this->PublishStop(); };
    process_sleep_ = true;
    sleep_start_ = aero::time::now();
  }

  _res.c_x = clusters_[nearest_cluster_id].center.x;
  _res.c_y = clusters_[nearest_cluster_id].center.y;
  _res.c_z = clusters_[nearest_cluster_id].center.z;
  _res.max_x = clusters_[nearest_cluster_id].max_bound.x;
  _res.max_y = clusters_[nearest_cluster_id].max_bound.y;
  _res.max_z = clusters_[nearest_cluster_id].max_bound.z;
  _res.min_x = clusters_[nearest_cluster_id].min_bound.x;
  _res.min_y = clusters_[nearest_cluster_id].min_bound.y;
  _res.min_z = clusters_[nearest_cluster_id].min_bound.z;
  _res.points = clusters_[nearest_cluster_id].points;
  return true;
}


//////////////////////////////////////////////////
void ClusteredCloud::SetPublishStyle(std::function<void()> _func)
{
  this->publish_func_ = _func;
  this->default_func_ = _func;
}

//////////////////////////////////////////////////
void ClusteredCloud::PublishStop()
{
}

//////////////////////////////////////////////////
void ClusteredCloud::PublishAllClusters()
{
  std_msgs::Float32MultiArray p_msg;
  std_msgs::MultiArrayLayout layout;
  layout.dim.reserve(clouds_.size() + 1);
  std_msgs::MultiArrayDimension dim_head;
  dim_head.label = "info";
  dim_head.size = 0;
  dim_head.stride = clouds_.size();
  layout.dim.push_back(dim_head);
  for (unsigned int i = 0; i < clouds_.size(); ++i)
  {
    std_msgs::MultiArrayDimension dim;
    dim.label = "block" + std::to_string(i);
    dim.size = clouds_[i].size();
    dim.stride = 3;
    layout.dim.push_back(dim);
    int start_idx = p_msg.data.size();
    p_msg.data.resize(p_msg.data.size() + dim.size * dim.stride);
    for (unsigned int j = 0; j < clouds_[i].size(); ++j)
    {
      int point_idx = j * dim.stride;
      p_msg.data[start_idx + point_idx] = clouds_[i][j].x();
      p_msg.data[start_idx + point_idx + 1] = clouds_[i][j].y();
      p_msg.data[start_idx + point_idx + 2] = clouds_[i][j].z();
    }
  }

  p_msg.layout = layout;
  point_publisher_.publish(p_msg);
}

//////////////////////////////////////////////////
void ClusteredCloud::PublishLargestCluster()
{
  std_msgs::Float32MultiArray p_msg;
  std_msgs::MultiArrayLayout layout;
  layout.dim.reserve(2);
  std_msgs::MultiArrayDimension dim_head;
  dim_head.label = "info";
  dim_head.size = 0;
  std_msgs::MultiArrayDimension dim;
  dim.label = "block1";
  dim.stride = 3;

  if (clouds_.size() == 0)
  {
    dim_head.stride = 0;
    dim.size = 0;
    p_msg.data.resize(0);
  }
  else
  {
    dim_head.stride = 1;
    int max_id = 0;
    int largest_cluster = 0;
    for (unsigned int i = 0; i < clouds_.size(); ++i)
      if (clouds_[i].size() > largest_cluster)
      {
	max_id = i;
	largest_cluster = clouds_[i].size();
      }
    dim.size = clouds_[max_id].size();
    p_msg.data.resize(dim.size * dim.stride);
    for (unsigned int j = 0; j < clouds_[max_id].size(); ++j)
    {
      int point_idx = j * dim.stride;
      p_msg.data[point_idx] = clouds_[max_id][j].x();
      p_msg.data[point_idx + 1] = clouds_[max_id][j].y();
      p_msg.data[point_idx + 2] = clouds_[max_id][j].z();
    }
  }

  layout.dim.push_back(dim_head);
  layout.dim.push_back(dim);
  p_msg.layout = layout;
  point_publisher_.publish(p_msg);
}
