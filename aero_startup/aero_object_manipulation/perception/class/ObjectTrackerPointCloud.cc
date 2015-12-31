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
      nh_.subscribe("/stereo/points2", 1,
		    &ObjectTrackerPointCloud::SubscribePoints, this);
  get_object_ = nh_.serviceClient<aero_startup::BoxFromXYZ>(
      "/point_cloud/get_object_from_center");
  get_plane_color_ = nh_.serviceClient<aero_startup::ReturnRGB>(
      "/plane_detection/get_plane_color");
  sleep_service_ =
    nh_.advertiseService("/object_tracker/sleep",
			 &ObjectTrackerPointCloud::ProcessSleep, this);

  object_center_ = Eigen::Vector3f(0.0, 0.0, 0.0);

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
  auto start = aero::time::now();

  if (sleep_) return;

  // check if pseudo tf is ready
  if (fabs(base_to_eye_.position.x) < 0.001 &&
      fabs(base_to_eye_.position.y) < 0.001 &&
      fabs(base_to_eye_.position.z) < 0.001) // camera = base_link not expected
  {
    ROS_ERROR("invalid pseudo camera tf");
    return;
  }

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl, *raw);
  Eigen::Vector3f object_center(0.0, 0.0, 0.0);
  std::vector<aero::xyz> points;
  points.reserve(raw->points.size());
  int points_count = 0;

  bool initial_subscribe_flag = false;

  // object = base is not valid,
  // when this is true, conditions are not initialized
  if (fabs(object_center_.x()) < 0.001 &&
      fabs(object_center_.y()) < 0.001 &&
      fabs(object_center_.z()) < 0.001)
  {
    ROS_WARN("initializing tracking object data");
    initial_subscribe_flag = true;
    this->InitializeTrackingObjectData(raw);
  }

  // only add points that are in bounding box
  for (unsigned int i = 0; i < raw->points.size(); ++i)
  {
    Eigen::Vector3f p(raw->points[i].x, raw->points[i].y, raw->points[i].z);
    if (!this->BoundingConditions(p)) continue;

    aero::rgb c = {raw->points[i].r, raw->points[i].g, raw->points[i].b};
    float dist1 =
        aero::colors::distance(object_color_, aero::colors::rgb2lab(c));
    float dist2 =
        aero::colors::distance(plane_color_, aero::colors::rgb2lab(c));
    if (dist2 - dist1 > 5.0)
    {
      // ROS_INFO("color is %d %d %d", c.r, c.g, c.b);
      // ROS_INFO("dist to object %f", dist1);
      // ROS_INFO("dist to plane %f", dist2);
      object_center += p;
      points.push_back({p.x(), p.y(), p.z()});
      ++points_count;
    }
  }
  object_center /= points_count;
  points.resize(points_count);

  if (points_count == 0)
  {
    ROS_WARN("no points detected in tracker");
    return;
  }

  if (initial_subscribe_flag) // initial subscribe ends here
  {
    object_center_ = object_center;
    return;
  }

  Eigen::Vector3f translation =
      {object_center.x() - object_center_.x(),
       object_center.y() - object_center_.y(),
       object_center.z() - object_center_.z()};
  object_center_ = object_center;

  ROS_WARN("translation : %f %f %f",
	   translation.x(), translation.y(), translation.z());

  object_.max_bound =
      {object_.max_bound.x + translation.x(),
       object_.max_bound.y + translation.y(),
       object_.max_bound.z + translation.z()};
  object_.min_bound =
      {object_.min_bound.x + translation.x(),
       object_.min_bound.y + translation.y(),
       object_.min_bound.z + translation.z()};
  object_.center =
      {object_.center.x + translation.x(),
       object_.center.y + translation.y(),
       object_.center.z + translation.z()};

  std_msgs::Float32MultiArray p_msg;
  std_msgs::MultiArrayLayout layout;
  std_msgs::MultiArrayDimension dim_head;
  dim_head.label = "info";
  dim_head.size = 0;
  dim_head.stride = 1;
  layout.dim.push_back(dim_head);
  std_msgs::MultiArrayDimension dim;
  dim.label = "block1";
  dim.size = points.size();
  dim.stride = 3;
  layout.dim.push_back(dim);
  p_msg.data.resize(dim.size * dim.stride);
  for (unsigned int j = 0; j < points.size(); ++j)
  {
    int point_idx = j * dim.stride;
    p_msg.data[point_idx] = points[j].x;
    p_msg.data[point_idx + 1] = points[j].y;
    p_msg.data[point_idx + 2] = points[j].z;
  }

  p_msg.layout = layout;
  points_publisher_.publish(p_msg);

  ROS_WARN("track time : %f", aero::time::ms(aero::time::now() - start));
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
    std::string wrt;
    std::istringstream iss(_req.message);
    iss >> srv.request.x;
    iss >> srv.request.y;
    iss >> srv.request.z;
    iss >> wrt;
    ROS_INFO("looking for target : %f %f %f %s",
	     srv.request.x, srv.request.y, srv.request.z, wrt.c_str());
    if (wrt == "world") // target position given in world coordinates
    {
      // check if pseudo tf is ready
      if (fabs(base_to_eye_.position.x) < 0.001 &&
	  fabs(base_to_eye_.position.y) < 0.001 &&
	  fabs(base_to_eye_.position.z) < 0.001)
	ROS_WARN("invalid pseudo camera tf, tracking might fail");
      Eigen::Quaternionf base_to_eye_q =
	  Eigen::Quaternionf(base_to_eye_.orientation.x,
			     base_to_eye_.orientation.y,
			     base_to_eye_.orientation.z,
			     base_to_eye_.orientation.w);
      Eigen::Vector3f request_in_camera_pos =
	  base_to_eye_q.inverse() * Eigen::Vector3f(srv.request.x,
						    srv.request.y,
						    srv.request.z);
      // only camera coords is accepted for request
      srv.request.x = request_in_camera_pos.x();
      srv.request.y = request_in_camera_pos.y();
      srv.request.z = request_in_camera_pos.z();
    }
    srv.request.kill_spin = true;
    // get object
    if (!get_object_.call(srv))
    {
      ROS_ERROR("unexpected call fail to service get object");
      _res.status = aero::status::fatal;
      sleep_ = true;
      return true;
    }
    // get plane color
    aero_startup::ReturnRGB srv2;
    if (!get_plane_color_.call(srv2))
    {
      ROS_ERROR("unexpected call fail to service get plane color");
      _res.status = aero::status::fatal;
      sleep_ = true;
      return true;
    }
    object_ =
        {{srv.response.c_x, srv.response.c_y, srv.response.c_z},
	 {srv.response.max_x, srv.response.max_y, srv.response.max_z},
	 {srv.response.min_x, srv.response.min_y, srv.response.min_z},
	 srv.response.points};
    object_center_ = Eigen::Vector3f(0.0, 0.0, 0.0);
    aero::rgb plane_rgb =
        {srv2.response.r, srv2.response.g, srv2.response.b};
    plane_color_ = aero::colors::rgb2lab(plane_rgb);
    ROS_WARN("activating object tracking");
  }
  else ROS_WARN("sleeping object tracking");

  _res.status = aero::status::success;
  return true;
}

//////////////////////////////////////////////////
void ObjectTrackerPointCloud::InitializeTrackingObjectData(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _raw)
{
  std::vector<int> center_points_id(10, 0);
  int middle_idx_c = static_cast<int>(center_points_id.size() * 0.5);
  std::vector<float> to_center_minimal_distances(center_points_id.size(),
						 100000);

  for (unsigned int i = 0; i < _raw->points.size(); ++i)
  {
    Eigen::Vector3f p(_raw->points[i].x, _raw->points[i].y, _raw->points[i].z);
    if (!this->BoundingConditions(p)) continue;

    // check if p is a most-likely center point
    float c_distance = std::pow(p.x() - object_.center.x, 2) +
        std::pow(p.y() - object_.center.y, 2) +
        std::pow(p.z() - object_.center.z, 2);
    for (unsigned int j = 0; j < center_points_id.size(); ++j)
      if (c_distance <= to_center_minimal_distances[j])
      {
	for (unsigned int k = center_points_id.size() - 1; k > j; --k)
	{
	  to_center_minimal_distances[k] = to_center_minimal_distances[k - 1];
	  center_points_id[k] = center_points_id[k - 1];
	}
	to_center_minimal_distances[j] = c_distance;
	center_points_id[j] = i;
	break;
      }
  }

  aero::lab white = {100, 0.005, -0.01};

  // create center Set
  std::vector<std::vector<rgb_dist> > grouped_samples(11);
  for (unsigned int i = 0; i < center_points_id.size(); ++i)
  {
    int idx = center_points_id[i];
    aero::rgb c =
        {_raw->points[idx].r, _raw->points[idx].g, _raw->points[idx].b};
    float distance =
        aero::colors::distance(aero::colors::rgb2lab(c), white);
    aero::rgb_dist p = {c, distance};
    grouped_samples[static_cast<int>(distance / 10)].push_back(p);
  }

  int largest_group = 0;
  int largest_group_size = 0;
  for (unsigned int i = 0; i < 11; ++i)
    if (grouped_samples[i].size() > largest_group_size)
    {
      largest_group = i;
      largest_group_size = grouped_samples[i].size();
    }

  std::sort(grouped_samples[largest_group].begin(),
	    grouped_samples[largest_group].end());
  aero::hsi center_color =
      aero::colors::rgb2hsi(
          grouped_samples[largest_group][largest_group_size * 0.5].color);
  object_color_ =
      aero::colors::rgb2lab(
          grouped_samples[largest_group][largest_group_size * 0.5].color);

  ROS_INFO("object color %f %f %f",
	   object_color_.l, object_color_.a, object_color_.b);
  ROS_INFO("plane color %f %f %f",
  	   plane_color_.l, plane_color_.a, plane_color_.b);
}
