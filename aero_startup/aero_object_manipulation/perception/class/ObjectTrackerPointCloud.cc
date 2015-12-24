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

  // check if pseudo tf is ready
  if (fabs(base_to_eye_.position.x) < 0.001 &&
      fabs(base_to_eye_.position.y) < 0.001 &&
      fabs(base_to_eye_.position.z) < 0.001) // camera = base_link not expected
  {
    ROS_ERROR("invalid pseudo camera tf");
    return;
  }

  // rotate bounding box to box in world coords
  Eigen::Quaternionf base_to_eye_q =
      Eigen::Quaternionf(base_to_eye_.orientation.x,
			 base_to_eye_.orientation.y,
			 base_to_eye_.orientation.z,
			 base_to_eye_.orientation.w);
  // set rotated bounding points
  std::vector<Eigen::Vector3f> rotated_bounding_points =
      {base_to_eye_q.inverse() *
       Eigen::Vector3f((object_.max_bound.x - object_.center.x) * 0.8,
		       (object_.max_bound.y - object_.center.y) * 0.8,
		       (object_.max_bound.z - object_.center.z) * 0.8),
       base_to_eye_q.inverse() *
       Eigen::Vector3f((object_.min_bound.x - object_.center.x) * 0.8,
		       (object_.min_bound.y - object_.center.y) * 0.8,
		       (object_.min_bound.z - object_.center.z) * 0.8)};

  // identify which points are max and which are min in world coords
  int max_x_point_id = 0, min_x_point_id = 0;
  int max_y_point_id = 0, min_y_point_id = 0;
  int max_z_point_id = 0, min_z_point_id = 0;
  for (unsigned int i = 0; i < rotated_bounding_points.size(); ++i)
  {
    Eigen::Vector3f bounding_point_world =
        base_to_eye_q * rotated_bounding_points[i];
    if (bounding_point_world.x() >= 0.0) max_x_point_id = i;
    else if (bounding_point_world.x() <= 0.0) min_x_point_id = i;
    if (bounding_point_world.y() >= 0.0) max_y_point_id = i;
    else if (bounding_point_world.y() <= 0.0) min_y_point_id = i;
    if (bounding_point_world.z() >= 0.0) max_z_point_id = i;
    else if (bounding_point_world.z() <= 0.0) min_z_point_id = i;
  }
  // calculate plane norms in camera coord
  Eigen::Vector3f plane_xp_norm =
    base_to_eye_q.inverse() * Eigen::Vector3f(1, 0, 0);
  Eigen::Vector3f plane_xm_norm =
    base_to_eye_q.inverse() * Eigen::Vector3f(-1, 0, 0);
  Eigen::Vector3f plane_yp_norm =
    base_to_eye_q.inverse() * Eigen::Vector3f(0, 1, 0);
  Eigen::Vector3f plane_ym_norm =
    base_to_eye_q.inverse() * Eigen::Vector3f(0, -1, 0);
  Eigen::Vector3f plane_zp_norm =
    base_to_eye_q.inverse() * Eigen::Vector3f(0, 0, 1);
  Eigen::Vector3f plane_zm_norm =
    base_to_eye_q.inverse() * Eigen::Vector3f(0, 0, -1);

  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(*_msg, pcl);
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl, *raw);
  std::vector<Eigen::Vector3f> object_points;
  object_points.reserve(raw->points.size());
  Eigen::Vector3f object_center(0.0, 0.0, 0.0);
  Eigen::Vector3f c(object_.center.x, object_.center.y, object_.center.z);

  // obly add points that are in world bounding plane
  for (unsigned int i = 0; i < raw->points.size(); ++i)
  {
    Eigen::Vector3f p(raw->points[i].x, raw->points[i].y, raw->points[i].z);
    if (plane_xp_norm.dot(p - c - rotated_bounding_points[max_x_point_id]) < 0 &&
	plane_xm_norm.dot(p - c - rotated_bounding_points[min_x_point_id]) < 0 &&
	plane_yp_norm.dot(p - c - rotated_bounding_points[max_y_point_id]) < 0 &&
	plane_ym_norm.dot(p - c - rotated_bounding_points[min_y_point_id]) < 0 &&
	plane_zp_norm.dot(p - c - rotated_bounding_points[max_z_point_id]) < 0 &&
	plane_zm_norm.dot(p - c - rotated_bounding_points[min_z_point_id]) < 0)
    {
      object_points.push_back(p);
      object_center += p;
    }
  }
  object_points.resize(object_points.size());
  object_center /= object_points.size();

  if (object_points.size() == 0) return;

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
    ROS_WARN("activating object tracking");
  }
  else ROS_WARN("sleeping object tracking");

  _res.status = aero::status::success;
  return true;
}
