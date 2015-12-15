#include "aero_object_manipulation/perception/class/PointCloudSensor.hh"
#include "aero_object_manipulation/perception/class/ObjectTrackerPointCloud.hh"

using namespace aero;
using namespace perception;

//////////////////////////////////////////////////
ObjectTrackerPointCloud::ObjectTrackerPointCloud(ros::NodeHandle _nh)
    : PointCloudSensor(_nh)
{
  object_ = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 0};

  point_cloud_listener_ =
      nh_.subscribe("/stereo/points2", 1000,
		    &ObjectTrackerPointCloud::SubscribePoints, this);
  get_object_ = nh_.serviceClient<aero_startup::BoxFromXYZ>(
      "/point_cloud/get_object_from_center");
  sleep_service_ =
    nh_.advertiseService("/object_tracker/sleep",
			 &ObjectTrackerPointCloud::ProcessSleep, this);

  sleep_ = true;
}

//////////////////////////////////////////////////
ObjectTrackerPointCloud::~ObjectTrackerPointCloud()
{
}

//////////////////////////////////////////////////
void ObjectTrackerPointCloud::SubscribePoints(
    const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  if (sleep_) return;

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl, *raw);
  std::vector<Eigen::Vector3f> object_points;
  object_points.reserve(raw->points.size());
  Eigen::Vector3f object_center(0.0, 0.0, 0.0);

  for (unsigned int i = 0; i < raw->points.size(); ++i)
    if ((raw->points[i].x > object_.min_bound.x) &&
	(raw->points[i].x < object_.max_bound.x) &&
	(raw->points[i].y > object_.min_bound.y) &&
	(raw->points[i].y < object_.max_bound.y) &&
	(raw->points[i].z > object_.min_bound.z) &&
	(raw->points[i].z < object_.max_bound.z))
    {
      Eigen::Vector3f p(raw->points[i].x, raw->points[i].y, raw->points[i].z);
      object_points.push_back(p);
      object_center += p;
    }
  object_points.resize(object_points.size());
  object_center /= object_points.size();

  ROS_INFO("object in : %f %f %f",
	   object_.center.x, object_.center.y, object_.center.z);

  object_.max_bound =
    {object_.max_bound.x + object_center.x() - object_.center.x,
     object_.max_bound.y + object_center.y() - object_.center.y,
     object_.max_bound.z + object_center.z() - object_.center.z};
  object_.min_bound =
    {object_.min_bound.x + object_center.x() - object_.center.x,
     object_.min_bound.y + object_center.y() - object_.center.y,
     object_.min_bound.z + object_center.z() - object_.center.z};
  object_.center = {object_center.x(), object_center.y(), object_center.z()};

  std::vector<Eigen::Vector3f> updated_points;
  updated_points.reserve(raw->points.size());  
  for (unsigned int i = 0; i < raw->points.size(); ++i)
    if ((raw->points[i].x > object_.min_bound.x) &&
	(raw->points[i].x < object_.max_bound.x) &&
	(raw->points[i].y > object_.min_bound.y) &&
	(raw->points[i].y < object_.max_bound.y) &&
	(raw->points[i].z > object_.min_bound.z) &&
	(raw->points[i].z < object_.max_bound.z))
    {
      Eigen::Vector3f p(raw->points[i].x, raw->points[i].y, raw->points[i].z);
      updated_points.push_back(p);
    }
  updated_points.resize(updated_points.size());

  object_.points = updated_points.size();

  std_msgs::Float32MultiArray p_msg;
  std_msgs::MultiArrayLayout layout;
  std_msgs::MultiArrayDimension dim_head;
  dim_head.label = "info";
  dim_head.size = 0;
  dim_head.stride = 1;
  layout.dim.push_back(dim_head);
  std_msgs::MultiArrayDimension dim;
  dim.label = "block1";
  dim.size = updated_points.size();
  dim.stride = 3;
  layout.dim.push_back(dim);
  p_msg.data.resize(dim.size * dim.stride);
  for (unsigned int j = 0; j < updated_points.size(); ++j)
  {
    int point_idx = j * dim.stride;
    p_msg.data[point_idx] = updated_points[j].x();
    p_msg.data[point_idx + 1] = updated_points[j].y();
    p_msg.data[point_idx + 2] = updated_points[j].z();
  }

  p_msg.layout = layout;
  points_publisher_.publish(p_msg);
}

//////////////////////////////////////////////////
bool ObjectTrackerPointCloud::ProcessSleep(
    aero_startup::ProcessSleep::Request  &_req,
    aero_startup::ProcessSleep::Response &_res)
{
  ROS_WARN("in tracker : %d", _req.sleep);
  sleep_ = _req.sleep;

  if (!sleep_) // when triggered on
  {
    aero_startup::BoxFromXYZ srv;
    // parse message
    std::istringstream iss(_req.message);
    char c;
    iss >> srv.request.x >> c >> srv.request.y >> c >> srv.request.z;
    srv.request.kill_spin = true;
    // set object
    if (!get_object_.call(srv))
    {
      ROS_ERROR("unexpected call fail to service");
      _res.status = aero::status::fatal;
      sleep_ = true;
      return true;
    }
    object_ =
        {{srv.response.c_x, srv.response.c_y, srv.response.c_z},
	 {srv.response.max_x, srv.response.max_y, srv.response.max_z},
	 {srv.response.min_x, srv.response.min_y, srv.response.min_z},
	 srv.response.points};
    ROS_WARN("activating object tracking");
  }
  else ROS_WARN("sleeping object tracking");

  _res.status = aero::status::success;
  return true;
}
