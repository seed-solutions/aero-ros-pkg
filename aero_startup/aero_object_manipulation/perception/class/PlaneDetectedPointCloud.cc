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

  space_min_ = {-0.5, -1.0, 0.8};
  space_max_ = {0.5, 1.0, 5.0};

  point_cloud_listener_ =
      nh_.subscribe("/stereo/points2", 1000,
		    &PlaneDetectedPointCloud::SubscribePoints, this);
  sleep_service_ =
      nh_.advertiseService("/plane_detection/sleep",
			   &PointCloudSensor::ProcessSleep,
			   dynamic_cast<PointCloudSensor*>(this));

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
  // for (int i = plane_idx[plane_idx.size()-1]; i < num_of_regions; ++i)
  for (unsigned int i = start_from; i < num_of_regions; ++i)
  {
    if (vertices_block[i].size() == 0) continue;

    // ROS_WARN("block %d has %d points", i, vertices_block[i].size());
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

  // get the size of plane

  // cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  // cloud_->points.reserve(plane_points.size());

  Eigen::Vector3f plane_center(0, 0, 0);
  for (unsigned int j = 0; j < plane_points.size(); ++j)
  {
    plane_center += Eigen::Vector3f(plane_points[j].x,
				    plane_points[j].y,
				    plane_points[j].z);
    // cloud_->points.push_back(pcl::PointXYZ(plane_points[j].x,
    // 					   plane_points[j].y,
    // 					   plane_points[j].z));
  }
  plane_center = plane_center * (1.0 / plane_points.size());

  Eigen::Vector3f plane_variance(0, 0, 0);
  for (unsigned int j = 0; j < plane_points.size(); ++j)
  {
    Eigen::Vector3f variance = Eigen::Vector3f(
	(plane_points[j].x - plane_center.x()) *
            (plane_points[j].x - plane_center.x()),
        (plane_points[j].y - plane_center.y()) *
            (plane_points[j].y - plane_center.y()),
        (plane_points[j].z - plane_center.z()) *
            (plane_points[j].z - plane_center.z()));
    plane_variance += variance;
  }
  plane_variance = plane_variance * (4.0 / plane_points.size()); // cut 5%

  // get plane color

  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_int_distribution<int> rand(0, plane_points.size());
  int num_of_samples;
  if (plane_points.size() < 50) num_of_samples = plane_points.size();
  else num_of_samples = 50; // use only 50 points if plane is big
  std::vector<aero::rgb_dist> sample_points(num_of_samples);

  aero::lab white = {100, 0.005, -0.01};

  for (unsigned int j = 0; j < num_of_samples; ++j)
  {
    int idx = plane_points[rand(mt)].id;
    aero::rgb c = {raw->points[j].r, raw->points[j].g, raw->points[j].b};
    sample_points[j].color = c;
    sample_points[j].dist =
        aero::colors::distance(aero::colors::rgb2lab(c), white);
  }

  std::sort(sample_points.begin(), sample_points.end());
  aero::hsi main_color =
      aero::colors::rgb2hsi(sample_points[num_of_samples * 0.5].color);
  aero::lab main_lab =
      aero::colors::rgb2lab(sample_points[num_of_samples * 0.5].color);
  aero::hsi edge_color1 = aero::colors::rgb2hsi(sample_points[0].color);
  aero::hsi edge_color2 =
      aero::colors::rgb2hsi(sample_points[num_of_samples - 1].color);
  // table color minimum hsi bound
  aero::hsi color_min =
      {std::min({main_color.h, edge_color1.h, edge_color2.h}) - 5,
       std::min({main_color.s, edge_color1.s, edge_color2.s}) - 20,
       std::min({main_color.i, edge_color1.i, edge_color2.i}) - 20};
  // table color maximum hsi bound
  aero::hsi color_max =
      {std::max({main_color.h, edge_color1.h, edge_color2.h}) + 5,
       std::max({main_color.s, edge_color1.s, edge_color2.s}) + 20,
       std::max({main_color.i, edge_color1.i, edge_color2.i}) + 20};

  // ROS_WARN("color range %d %d %d ~ %d %d %d",
  // 	   color_min.h, color_min.s, color_min.i,
  // 	   color_max.h, color_max.s, color_max.i);

  // get the points on plane

  vertices_.clear();
  vertices_.reserve(raw->points.size());
  if (max_points_when_shifted > max_points)
    for (unsigned int i = // objects higher than 0 cm
	   (max_index_when_shifted + 1 + 0.00 / height_range_per_region_);
	 i < vertices_shifted.size(); ++i)
      for (unsigned int j = 0; j < vertices_shifted[i].size(); ++j)
      {
	if ((vertices_shifted[i][j].x - plane_center.x()) *
	      (vertices_shifted[i][j].x - plane_center.x()) <
	    plane_variance.x() &&
	    // (vertices_shifted[i][j].y - plane_center.y()) *
	    //   (vertices_shifted[i][j].y - plane_center.y()) <
	    // plane_variance.y() &&
	    (vertices_shifted[i][j].z - plane_center.z()) *
	      (vertices_shifted[i][j].z - plane_center.z()) <
	    plane_variance.z())
	 {
	   aero::rgb c = {raw->points[vertices_shifted[i][j].id].r,
			  raw->points[vertices_shifted[i][j].id].g,
			  raw->points[vertices_shifted[i][j].id].b};
	   if (!aero::colors::compare(c, color_min, color_max))
	     // not a noise from plane
	     vertices_.push_back(Eigen::Vector3f(vertices_shifted[i][j].x,
						 vertices_shifted[i][j].y,
						 vertices_shifted[i][j].z));
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
	    // (vertices_block[i][j].y - plane_center.y()) *
	    //   (vertices_block[i][j].y - plane_center.y()) <
	    // plane_variance.y() &&
	    (vertices_block[i][j].z - plane_center.z()) *
	      (vertices_block[i][j].z - plane_center.z()) <
	    plane_variance.z())
	{
	   aero::rgb c = {raw->points[vertices_block[i][j].id].r,
			  raw->points[vertices_block[i][j].id].g,
			  raw->points[vertices_block[i][j].id].b};
	   if (!aero::colors::compare(c, color_min, color_max))
	     // not a noise from plane
	     vertices_.push_back(Eigen::Vector3f(vertices_block[i][j].x,
						 vertices_block[i][j].y,
						 vertices_block[i][j].z));
	}
      }
  vertices_.resize(vertices_.size());

  timing_ = true;

  std_msgs::Float32MultiArray p_msg;
  std_msgs::MultiArrayLayout layout;
  std_msgs::MultiArrayDimension dim_head;
  dim_head.label = "info";
  dim_head.size = 0; // is not used
  dim_head.stride = 1; // number of blocks
  layout.dim.push_back(dim_head);
  std_msgs::MultiArrayDimension dim;
  dim.label = "block1";
  dim.size = vertices_.size(); // number of points
  dim.stride = 3; // number of data for each point
  layout.dim.push_back(dim);
  p_msg.layout = layout;
  p_msg.data.reserve(dim.stride * dim.size);
  for (unsigned int i = 0; i < vertices_.size(); ++i)
  {
    p_msg.data.push_back(vertices_[i].x());
    p_msg.data.push_back(vertices_[i].y());
    p_msg.data.push_back(vertices_[i].z());
  }
  points_publisher_.publish(p_msg);

  // now we have the points on the plane

  cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  cloud_->points.reserve(vertices_.size());

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
