#include "aero_object_manipulation/perception/class/ClusteredCloud.hh"

using namespace aero;
using namespace common;

//////////////////////////////////////////////////
ClusteredCloud::ClusteredCloud(ros::NodeHandle _nh) : nh_(_nh)
{
  point_publisher_ =
      nh_.advertise<std_msgs::Float32MultiArray>("/cluster_cloud/points", 1000);

  subscriber_ = nh_.subscribe("/point_cloud/objects", 1000,
			      &ClusteredCloud::Subscribe, this);

  this->publish_func_ = [=](){ this->PublishAllClusters(); };
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
  clouds_.clear();
  int num_of_clusters = _points->layout.dim[0].stride;
  clouds_.reserve(num_of_clusters);

  int data_idx = 0;
  for (unsigned int i = 1; i < num_of_clusters; ++i)
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
}

//////////////////////////////////////////////////
void ClusteredCloud::SetPublishStyle(std::function<void()> _func)
{
  this->publish_func_ = _func;
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
  dim_head.stride = 1;
  layout.dim.push_back(dim_head);
  std_msgs::MultiArrayDimension dim;
  dim.label = "block1";
  dim.stride = 3;

  if (clouds_.size() == 0)
  {
    dim.size = 0;
    p_msg.data.resize(0);
  }
  else
  {
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
      p_msg.data[point_idx + j] = clouds_[max_id][j].x();
      p_msg.data[point_idx + j + 1] = clouds_[max_id][j].y();
      p_msg.data[point_idx + j + 2] = clouds_[max_id][j].z();
    }
  }

  layout.dim.push_back(dim);
  p_msg.layout = layout;
  point_publisher_.publish(p_msg);
}
