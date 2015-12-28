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
  sleep_service_ =
    nh_.advertiseService("/object_tracker/sleep",
			 &ObjectTrackerPointCloud::ProcessSleep, this);

  object_center_ = Eigen::Vector3f(0.0, 0.0, 0.0);
  trim_color_min_ = {0, 0, 0};
  trim_color_max_ = {0, 0, 0};

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
    aero::rgb c = {raw->points[i].r, raw->points[i].g, raw->points[i].b};
    if (this->BoundingConditions(p) &&
	!aero::colors::compare(c, trim_color_min_, trim_color_max_))
    {
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

  ROS_WARN("time : %f", aero::time::ms(aero::time::now() - start));
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
    object_center_ = Eigen::Vector3f(0.0, 0.0, 0.0);
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
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<float> rand_y(
      object_.min_bound.y, object_.max_bound.y);

  // create random sample outliers to get boundary points
  std::vector<aero::xyz> random_x_outliers(10);
  int middle_idx_b = static_cast<int>(random_x_outliers.size() * 0.5);
  for (unsigned int i = 0; i < middle_idx_b; ++i)
    random_x_outliers[i] = {object_.max_bound.x + 1.0, rand_y(mt), 0.0};
  for (unsigned int i = middle_idx_b; i < random_x_outliers.size(); ++i)
    random_x_outliers[i] = {object_.max_bound.x - 1.0, rand_y(mt), 0.0};

  // initialize boundary and center points
  std::vector<int> boundary_points_id(random_x_outliers.size(), 0);
  std::vector<float> to_bound_minimal_distances(random_x_outliers.size(),
						100000);
  std::vector<int> center_points_id(10, 0);
  int middle_idx_c = static_cast<int>(center_points_id.size() * 0.5);
  std::vector<float> to_center_minimal_distances(center_points_id.size(),
						 100000);

  for (unsigned int i = 0; i < _raw->points.size(); ++i)
  {
    Eigen::Vector3f p(_raw->points[i].x, _raw->points[i].y, _raw->points[i].z);
    if (!this->BoundingConditions(p)) continue;

    // check if p is a most-likely boundary point
    for (unsigned int j = 0; j < random_x_outliers.size(); ++j)
    {
      float b_distance = std::pow(p.x() - random_x_outliers[j].x, 2) +
	  std::pow(p.y() - random_x_outliers[j].y, 2);
      if (b_distance < to_bound_minimal_distances[j])
      {
	boundary_points_id[j] = i;
	to_bound_minimal_distances[j] = b_distance;
      }
    }

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

  // create boundary Set
  std::vector<aero::rgb_dist> boundary_points(random_x_outliers.size());
  for (unsigned int i = 0; i < random_x_outliers.size(); ++i)
  {
    int idx = boundary_points_id[i];
    aero::rgb c =
        {_raw->points[idx].r, _raw->points[idx].g, _raw->points[idx].b};
    boundary_points[i].color = c;
    boundary_points[i].dist =
        aero::colors::distance(aero::colors::rgb2lab(c), white);
  }

  // create center Set
  std::vector<aero::rgb_dist> center_points(center_points_id.size());
  for (unsigned int i = 0; i < center_points_id.size(); ++i)
  {
    int idx = center_points_id[i];
    aero::rgb c =
        {_raw->points[idx].r, _raw->points[idx].g, _raw->points[idx].b};
    center_points[i].color = c;
    center_points[i].dist =
        aero::colors::distance(aero::colors::rgb2lab(c), white);
  }

  // sort each Set by color distance to white
  std::sort(boundary_points.begin(), boundary_points.end());
  std::sort(center_points.begin(), center_points.end());

  // get main color of each set (sorted median)
  aero::rgb center_color = center_points[middle_idx_c].color;
  aero::lab center_lab = aero::colors::rgb2lab(center_color);
  aero::hsi center_hsi = aero::colors::rgb2hsi(center_color);
  aero::hsi center_hsi_min =
      {center_hsi.h - 20, center_hsi.s - 50, center_hsi.i - 50};
  aero::hsi center_hsi_max =
      {center_hsi.h + 20, center_hsi.s + 50, center_hsi.i + 50};
  aero::rgb boundary_color = boundary_points[middle_idx_b].color;
  aero::lab boundary_lab = aero::colors::rgb2lab(boundary_color);
  aero::hsi boundary_hsi = aero::colors::rgb2hsi(boundary_color);
  aero::hsi boundary_hsi_min =
      {boundary_hsi.h - 20, boundary_hsi.s - 50, boundary_hsi.i - 50};
  aero::hsi boundary_hsi_max =
      {boundary_hsi.h + 20, boundary_hsi.s + 50, boundary_hsi.i + 50};

  // re-create boundary Set according to main color
  std::vector<aero::rgb_dist> boundary_points_re;
  boundary_points_re.reserve(boundary_points.size());
  for (unsigned int i = 0; i < boundary_points.size(); ++i)
    if (aero::colors::compare(
            boundary_points[i].color, boundary_hsi_min, boundary_hsi_max))
    {
      aero::rgb_dist p;
      p.color = boundary_points[i].color;
      p.dist =
	  aero::colors::distance(aero::colors::rgb2lab(p.color), boundary_lab);
      boundary_points_re.push_back(p);
    }
  boundary_points_re.resize(boundary_points_re.size());

  // re-create center Set according to main color
  std::vector<aero::rgb_dist> center_points_re;
  center_points_re.reserve(center_points.size());
  for (unsigned int i = 0; i < center_points.size(); ++i)
    if (aero::colors::compare(
            center_points[i].color, center_hsi_min, center_hsi_max))
    {
      aero::rgb_dist p;
      p.color = center_points[i].color;
      p.dist =
	  aero::colors::distance(aero::colors::rgb2lab(p.color), center_lab);
      center_points_re.push_back(p);
    }
  center_points_re.resize(center_points_re.size());

  // sort each Set by color distance to main color
  std::sort(boundary_points_re.begin(), boundary_points_re.end());
  std::sort(center_points_re.begin(), center_points_re.end());

  int b = boundary_points_re.size() - 1;
  int c = center_points_re.size() - 1;

  std::function<void()> reset_bounds = [&]()
  {
    aero::hsi limit = aero::colors::rgb2hsi(boundary_points_re[b].color);
    limit = {abs(limit.h - boundary_hsi.h) + 2,
	     abs(limit.s - boundary_hsi.s) + 5,
	     abs(limit.i - boundary_hsi.i) + 5};
    trim_color_min_ = {boundary_hsi.h - limit.h,
		       // boundary_hsi.s - limit.s,
		       // boundary_hsi.i - limit.i};
		       0, // don't care saturation
		       0}; // don't care intensity
    trim_color_max_ = {boundary_hsi.h + limit.h,
		       // boundary_hsi.s + limit.s,
		       // boundary_hsi.i + limit.i};
		       255, // don't care saturation
		       255}; // don't care intensity
  };

  // reset boundary hsi so that at least half of center points are not included
  reset_bounds();
  while (b >= 0)
  {
    if (!aero::colors::compare(
            center_points_re[c].color, trim_color_min_, trim_color_max_))
      break;

    --b; // shrink boundary
    if (c > middle_idx_c) --c; // try excluding far colored center points
    reset_bounds();
  }

  ROS_WARN("bound colors are : %d %d %d ~ %d %d %d",
	   trim_color_min_.h, trim_color_min_.s, trim_color_min_.i,
	   trim_color_max_.h, trim_color_max_.s, trim_color_max_.i);
}
