#include "aero_sensors/XtionInterface.hh"

using namespace xtion;
using namespace interface;

//////////////////////////////////////////////////
XtionInterface::XtionInterface(ros::NodeHandle _nh)
  : nh_(_nh), depth_spinner_(1, &depth_queue_), image_spinner_(1, &image_queue_),
    depth_width_(640)
{
  depth_ops_ =
    ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
        "/xtion/depth_registered/points",
        1000,
        boost::bind(&XtionInterface::DepthCallback, this, _1),
        ros::VoidPtr(),
        &depth_queue_);
  depth_sub_ = nh_.subscribe(depth_ops_);
  depth_spinner_.start();

  image_ops_ =
    ros::SubscribeOptions::create<sensor_msgs::Image>(
        "/xtion/rgb/image_raw",
        1000,
        boost::bind(&XtionInterface::ImageCallback, this, _1),
        ros::VoidPtr(),
        &image_queue_);
  image_sub_ = nh_.subscribe(image_ops_);
  image_spinner_.start();

  // wait for connection to openni
  usleep(2000 * 1000);
}

//////////////////////////////////////////////////
XtionInterface::~XtionInterface()
{
}

//////////////////////////////////////////////////
sensor_msgs::PointCloud2 XtionInterface::ReadPoints()
{
  depth_mutex_.lock();
  auto res = depth_;
  depth_mutex_.unlock();

  return res;
}

//////////////////////////////////////////////////
sensor_msgs::Image XtionInterface::ReadImage()
{
  image_mutex_.lock();
  auto res = image_;
  image_mutex_.unlock();

  return res;
}

//////////////////////////////////////////////////
std::vector<sensor_msgs::RegionOfInterest> XtionInterface::ImageBounds
(std::vector<std::array<int, 4> > _depth_indicies)
{
  std::vector<sensor_msgs::RegionOfInterest> result;
  result.reserve(_depth_indicies.size());

  for (auto it = _depth_indicies.begin(); it != _depth_indicies.end(); ++it) {
    // min x
    int depth0_y = it->at(0) / depth_width_;
    int depth0_x = it->at(0) - depth0_y * depth_width_;
    int pixel0_x = depth0_x;

    // min y
    int depth1_y = it->at(1) / depth_width_;
    int pixel1_y = depth1_y;

    // max_x
    int depth2_y = it->at(2) / depth_width_;
    int depth2_x = it->at(2) - depth2_y * depth_width_;
    int pixel2_x = depth2_x;

    // max_y
    int depth3_y = it->at(3) / depth_width_;
    int pixel3_y = depth3_y;

    sensor_msgs::RegionOfInterest image_bounds;
    image_bounds.x_offset = pixel0_x;
    image_bounds.y_offset = pixel1_y;
    image_bounds.width = pixel2_x - image_bounds.x_offset;
    image_bounds.height = pixel3_y - image_bounds.y_offset;

    result.push_back(image_bounds);
  }

  return result;
}

//////////////////////////////////////////////////
std::vector<geometry_msgs::Point> XtionInterface::ImageCenters
(std::vector<sensor_msgs::RegionOfInterest> _image_bounds)
{
  std::vector<geometry_msgs::Point> res;

  depth_mutex_.lock();
  for (auto it = _image_bounds.begin(); it != _image_bounds.end(); ++it) {
    int x = static_cast<int>(it->x_offset + 0.5 * it->width);
    int y = static_cast<int>(it->y_offset + 0.5 * it->height);
    int at = static_cast<int>((y * depth_.width + x) * depth_.point_step);

    geometry_msgs::Point p;
    uint8_t xbytes[4] =
      {depth_.data[at++], depth_.data[at++], depth_.data[at++], depth_.data[at++]};
    float xfloat;
    std::memcpy(&xfloat, &xbytes, 4);

    uint8_t ybytes[4] =
      {depth_.data[at++], depth_.data[at++], depth_.data[at++], depth_.data[at++]};
    float yfloat;
    std::memcpy(&yfloat, &ybytes, 4);

    uint8_t zbytes[4] =
      {depth_.data[at++], depth_.data[at++], depth_.data[at++], depth_.data[at++]};
    float zfloat;
    std::memcpy(&zfloat, &zbytes, 4);

    p.x = xfloat;
    p.y = yfloat;
    p.z = zfloat;

    res.push_back(p);
  }
  depth_mutex_.unlock();

  return res;
}

//////////////////////////////////////////////////
void XtionInterface::DepthCallback(const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  depth_mutex_.lock();
  depth_ = *_msg;
  depth_mutex_.unlock();
}

//////////////////////////////////////////////////
void XtionInterface::ImageCallback(const sensor_msgs::Image::ConstPtr& _msg)
{
  image_mutex_.lock();
  image_ = *_msg;
  image_mutex_.unlock();
}
