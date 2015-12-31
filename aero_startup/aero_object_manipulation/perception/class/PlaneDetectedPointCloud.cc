#include "aero_object_manipulation/perception/class/PlaneDetectedPointCloud.hh"

using namespace aero;
using namespace perception;

//////////////////////////////////////////////////
PlaneDetectedPointCloud::PlaneDetectedPointCloud(ros::NodeHandle _nh)
    : PointCloudSensor(_nh)
{
  height_range_per_region_ = 0.03;

  desk_plane_norm_ =
      Eigen::Quaternionf(1, 0, 0, 0) * Eigen::Vector3f(0, 0, 1);

  space_min_.x = -0.5;
  space_min_.y = -1.0;
  space_min_.z = 0.8;

  space_max_.x = 0.5;
  space_max_.y = 1.0;
  space_max_.z = 5.0;

  plane_color_.r = 0;
  plane_color_.g = 0;
  plane_color_.b = 0;

  point_cloud_listener_ =
      nh_.subscribe("/stereo/points2", 1,
		    &PlaneDetectedPointCloud::SubscribePoints, this);
  sleep_service_ =
      nh_.advertiseService("/plane_detection/sleep",
			   &PointCloudSensor::ProcessSleep,
			   dynamic_cast<PointCloudSensor*>(this));
  return_plane_color_service_ =
      nh_.advertiseService("/plane_detection/get_plane_color",
			   &PlaneDetectedPointCloud::ReturnPlaneRGB,this);

  pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
      "/visualized_object_pcl", 100);
  sleep_ = false;
}

//////////////////////////////////////////////////
PlaneDetectedPointCloud::~PlaneDetectedPointCloud()
{
}

//////////////////////////////////////////////////
void PlaneDetectedPointCloud::SubscribePoints(
    const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
#ifdef CXX11_SUPPORTED
  auto start = aero::time::now();
#else
  boost::chrono::system_clock::time_point start = aero::time::now();
#endif

  if (sleep_) return;

  // check if pseudo tf is ready
  if (fabs(base_to_eye_.position.x) < 0.001 &&
      fabs(base_to_eye_.position.y) < 0.001 &&
      fabs(base_to_eye_.position.z) < 0.001) // camera = base_link not expected
  {
    ROS_ERROR("invalid pseudo camera tf");
    return;
  }

  Eigen::Vector3f manipulation_space_max(
      space_max_.x, space_max_.y, space_max_.z);
  Eigen::Vector3f manipulation_space_min(
      space_min_.x, space_min_.y, space_min_.z);

  float height_range_total =
    (manipulation_space_max - manipulation_space_min).norm();
  int num_of_regions =
    height_range_total / height_range_per_region_ + 1;
  float lowest_height_level = -height_range_total;

  Eigen::Quaternionf base_to_eye_q =
    Eigen::Quaternionf(base_to_eye_.orientation.x,
		       base_to_eye_.orientation.y,
		       base_to_eye_.orientation.z,
		       base_to_eye_.orientation.w);
  desk_plane_norm_ =
    base_to_eye_q.inverse() * Eigen::Vector3f(0, 0, -1);

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr raw(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl, *raw);

  std::vector<std::vector<aero::point> > vertices_block(num_of_regions);
  std::vector<std::vector<aero::point> > vertices_shifted(num_of_regions);
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
      aero::point p = {static_cast<int>(i), point.x(), point.y(), point.z()};
      vertices_block[index].push_back(p);
      vertices_shifted[index_shifted].push_back(p);
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
  for (unsigned int i = start_from; i < num_of_regions; ++i)
  {
    if (vertices_block[i].size() == 0) continue;

    // ROS_INFO("block %d has %d points", i, vertices_block[i].size());
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

  std::vector<aero::point> plane_points;
  if (max_points_when_shifted > max_points)
    plane_points.assign(vertices_shifted[max_index_when_shifted].begin(),
			vertices_shifted[max_index_when_shifted].end());
  else
    plane_points.assign(vertices_block[max_index].begin(),
			vertices_block[max_index].end());

  // get plane color

#ifdef CXX11_SUPPORTED
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_int_distribution<int> rand(0, plane_points.size());
#else
  boost::random_device rd;
  boost::mt19937 mt(rd());
  boost::random::uniform_int_distribution<> rand(0, plane_points.size());
#endif
  int num_of_samples;
  if (plane_points.size() < 400) num_of_samples = plane_points.size();
  else num_of_samples = 400; // use only 400 points if plane is big
  std::vector<aero::rgb_dist> sample_points(num_of_samples);
  std::vector<aero::lab> sample_labs(num_of_samples);
  std::vector<std::vector<rgb_dist> > grouped_samples(21);

  aero::lab white;
  white.l = 100;
  white.a = 0.005;
  white.b = -0.01;

  for (unsigned int j = 0; j < num_of_samples; ++j)
  {
    int idx = plane_points[rand(mt)].id;
    aero::rgb c;
    c.r = raw->points[j].r;
    c.g = raw->points[j].g;
    c.b = raw->points[j].b;
    sample_points[j].color = c;
    sample_labs[j] = aero::colors::rgb2lab(c);
    sample_points[j].dist =
        aero::colors::distance(sample_labs[j], white);
    grouped_samples[static_cast<int>(sample_points[j].dist / 5)].
        push_back(sample_points[j]);
  }

  int largest_group = 0;
  int largest_group_size = 0;
  for (unsigned int i = 0; i < 21; ++i)
    if (grouped_samples[i].size() > largest_group_size)
    {
      largest_group = i;
      largest_group_size = grouped_samples[i].size();
    }

  Eigen::Vector3f average_color(0.0, 0.0, 0.0);
  for (unsigned int i = 0; i < grouped_samples[largest_group].size(); ++i)
    average_color +=
        Eigen::Vector3f(grouped_samples[largest_group][i].color.r,
			grouped_samples[largest_group][i].color.g,
			grouped_samples[largest_group][i].color.b);
  average_color /= grouped_samples[largest_group].size();
  plane_color_.r = average_color.x();
  plane_color_.g = average_color.y();
  plane_color_.b = average_color.z();
  aero::lab main_lab = aero::colors::rgb2lab(plane_color_);

  // ROS_INFO("main color %d %d %d",
  // 	   plane_color_.r, plane_color_.g, plane_color_.b);

  float dist_bound = 10.0;

  // get the size of plane by color

  // cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // cloud_->points.reserve(plane_points.size());

  Eigen::Vector3f plane_center(0, 0, 0);
  std::vector<int> valid_plane_points;
  valid_plane_points.reserve(plane_points.size());
  for (unsigned int j = 0; j < plane_points.size(); ++j)
  {
    aero::rgb c;
    c.r = raw->points[plane_points[j].id].r;
    c.g = raw->points[plane_points[j].id].g;
    c.b = raw->points[plane_points[j].id].b;
    if (aero::colors::distance(aero::colors::rgb2lab(c), main_lab) <
	dist_bound)
    {
      plane_center += Eigen::Vector3f(plane_points[j].x,
				      plane_points[j].y,
				      plane_points[j].z);
      valid_plane_points.push_back(j);
      // cloud_->points.push_back(pcl::PointXYZ(plane_points[j].x,
      // 					     plane_points[j].y,
      // 					     plane_points[j].z));
    }
  }
  valid_plane_points.resize(valid_plane_points.size());
  plane_center = plane_center * (1.0 / valid_plane_points.size());

  Eigen::Vector3f plane_variance(0, 0, 0);
  for (unsigned int j = 0; j < valid_plane_points.size(); ++j)
  {
    Eigen::Vector3f variance = Eigen::Vector3f(
	(plane_points[valid_plane_points[j]].x - plane_center.x()) *
            (plane_points[valid_plane_points[j]].x - plane_center.x()),
        (plane_points[valid_plane_points[j]].y - plane_center.y()) *
            (plane_points[valid_plane_points[j]].y - plane_center.y()),
        (plane_points[valid_plane_points[j]].z - plane_center.z()) *
            (plane_points[valid_plane_points[j]].z - plane_center.z()));
    plane_variance += variance;
  }
  // cut 5%
  plane_variance = plane_variance * (4.0 / valid_plane_points.size());

  // get the points on plane

  std::vector<int> on_plane_points;
  on_plane_points.reserve(raw->points.size());

  if (max_points_when_shifted > max_points)
    for (unsigned int i = // objects higher than 0 cm
	   (max_index_when_shifted + 1 + 0.00 / height_range_per_region_);
	 i < vertices_shifted.size(); ++i)
      for (unsigned int j = 0; j < vertices_shifted[i].size(); ++j)
      {
	if ((vertices_shifted[i][j].x - plane_center.x()) *
	      (vertices_shifted[i][j].x - plane_center.x()) <
	    plane_variance.x() &&
	    (vertices_shifted[i][j].z - plane_center.z()) *
	      (vertices_shifted[i][j].z - plane_center.z()) <
	    plane_variance.z())
	 {
	   aero::rgb c;
	   c.r = raw->points[vertices_shifted[i][j].id].r;
	   c.g = raw->points[vertices_shifted[i][j].id].g;
	   c.b = raw->points[vertices_shifted[i][j].id].b;
	   if (aero::colors::distance(aero::colors::rgb2lab(c), main_lab) >
	       dist_bound)
	     // not a noise from plane
	     on_plane_points.push_back(vertices_shifted[i][j].id);
	 }
      }
  else
    for (unsigned int i = // objects higher than 0 cm
	   (max_index + 1 + 0.00 / height_range_per_region_);
	 i < vertices_block.size(); ++i)
      for (unsigned int j = 0; j < vertices_block[i].size(); ++j)
      {
	if ((vertices_block[i][j].x - plane_center.x()) *
	      (vertices_block[i][j].x - plane_center.x()) <
	    plane_variance.x() &&
	    (vertices_block[i][j].z - plane_center.z()) *
	      (vertices_block[i][j].z - plane_center.z()) <
	    plane_variance.z())
	{
	  aero::rgb c;
	  c.r = raw->points[vertices_block[i][j].id].r;
	  c.g = raw->points[vertices_block[i][j].id].g;
	  c.b = raw->points[vertices_block[i][j].id].b;
	  if (aero::colors::distance(aero::colors::rgb2lab(c), main_lab) >
	      dist_bound)
	     // not a noise from plane
	    on_plane_points.push_back(vertices_block[i][j].id);
	}
      }
  on_plane_points.resize(on_plane_points.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr on_plane_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  on_plane_cloud->points.reserve(on_plane_points.size());
  for (unsigned int i = 0; i < on_plane_points.size(); ++i)
    on_plane_cloud->points.push_back(
        pcl::PointXYZ(raw->points[on_plane_points[i]].x,
		      raw->points[on_plane_points[i]].y,
		      raw->points[on_plane_points[i]].z));

  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->points.reserve(vertices_.size());
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(on_plane_cloud);
  sor.setMeanK(30);
  sor.setStddevMulThresh(0.5);
  sor.filter(*cloud_);

  vertices_.clear();
  vertices_.resize(cloud_->points.size());
  for (unsigned int i = 0; i < cloud_->points.size(); ++i)
    vertices_[i] = Eigen::Vector3f(cloud_->points[i].x,
				   cloud_->points[i].y,
				   cloud_->points[i].z);

  std_msgs::Float32MultiArray p_msg;
  std_msgs::MultiArrayLayout layout;
  std_msgs::MultiArrayDimension dim_head;
  dim_head.label = "info";
  dim_head.size = 4; // number of data in head
  dim_head.stride = 1; // number of blocks
  layout.dim.push_back(dim_head);
  std_msgs::MultiArrayDimension dim;
  dim.label = "block1";
  dim.size = vertices_.size(); // number of points
  dim.stride = 3; // number of data for each point
  layout.dim.push_back(dim);
  p_msg.layout = layout;
  p_msg.data.reserve(dim.stride * dim.size + dim_head.size);
  p_msg.data.push_back(plane_center.x());
  p_msg.data.push_back(plane_center.z());
  p_msg.data.push_back(plane_variance.x());
  p_msg.data.push_back(plane_variance.z());
  for (unsigned int i = 0; i < vertices_.size(); ++i)
  {
    p_msg.data.push_back(vertices_[i].x());
    p_msg.data.push_back(vertices_[i].y());
    p_msg.data.push_back(vertices_[i].z());
  }
  points_publisher_.publish(p_msg);

  // publish cloud for debug

  pcl::PCLPointCloud2 pcl_out;
  sensor_msgs::PointCloud2 msg;
  pcl::toPCLPointCloud2(*cloud_, pcl_out);
  pcl_conversions::fromPCL(pcl_out, msg);
  msg.header.frame_id = "ps4eye_frame";
  msg.header.stamp = ros::Time(0); // get possible recent
  pcl_pub_.publish(msg);

  // ROS_WARN("detect time : %f", aero::time::ms(aero::time::now() - start));
}

//////////////////////////////////////////////////
bool PlaneDetectedPointCloud::ReturnPlaneRGB(
    aero_startup::ReturnRGB::Request  &_req,
    aero_startup::ReturnRGB::Response &_res)
{
  _res.r = plane_color_.r;
  _res.g = plane_color_.g;
  _res.b = plane_color_.b;
}
