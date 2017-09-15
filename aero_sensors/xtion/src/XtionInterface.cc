#include "aero_sensors/XtionInterface.hh"

using namespace xtion;
using namespace interface;

//////////////////////////////////////////////////
XtionInterface::XtionInterface(ros::NodeHandle _nh,
                               const std::string& _depth_topic,
                               const std::string& _image_topic)
  : nh_(_nh), depth_spinner_(1, &depth_queue_), image_spinner_(1, &image_queue_),
    depth_width_(640), depth_height_(480)
{
  depth_ops_ =
    ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
        _depth_topic,
        1000,
        boost::bind(&XtionInterface::DepthCallback, this, _1),
        ros::VoidPtr(),
        &depth_queue_);
  depth_sub_ = nh_.subscribe(depth_ops_);
  depth_spinner_.start();

  image_ops_ =
    ros::SubscribeOptions::create<sensor_msgs::Image>(
        _image_topic,
        1000,
        boost::bind(&XtionInterface::ImageCallback, this, _1),
        ros::VoidPtr(),
        &image_queue_);
  image_sub_ = nh_.subscribe(image_ops_);
  image_spinner_.start();

  time_now_ = ros::Time::now();
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
sensor_msgs::PointCloud2 XtionInterface::ReadPoints(float _scale_x, float _scale_y)
{
  depth_mutex_.lock();
  auto res = sensor_msgs::PointCloud2();
  res.header.frame_id = depth_.header.frame_id;
  res.header.stamp = depth_.header.stamp;

  // note, field differs from original msg
  res.fields.resize(4, sensor_msgs::PointField());
  res.fields[0].name = "x";
  res.fields[0].offset = 0;
  res.fields[0].datatype = 7;
  res.fields[0].count = 1;
  res.fields[1].name = "y";
  res.fields[1].offset = 4;
  res.fields[1].datatype = 7;
  res.fields[1].count = 1;
  res.fields[2].name = "z";
  res.fields[2].offset = 8;
  res.fields[2].datatype = 7;
  res.fields[2].count = 1;
  res.fields[3].name = "rgb";
  res.fields[3].offset = 12;
  res.fields[3].datatype = 7;
  res.fields[3].count = 1;

  res.height = static_cast<int>(depth_.height * _scale_y);
  res.width = static_cast<int>(depth_.width * _scale_x);
  res.point_step = 16;
  res.row_step = res.point_step * res.width;
  res.is_dense = false;
  res.is_bigendian = false;

  int stride_x = static_cast<int>(1.0 / _scale_x) * depth_.point_step;
  int stride_y = static_cast<int>(1.0 / _scale_y);

  // compress point cloud
  res.data.resize(res.height * res.width * res.point_step);
  int row = 0;
  int at = 0;
  int j = 0;
  while (j < res.data.size()) {
    int tl = at; // top left
    int tr = at + stride_x - depth_.point_step; // top right
    int bl = at + depth_.row_step * (stride_y - 1); // bottom left
    int br = at + depth_.row_step * (stride_y - 1) + stride_x - depth_.point_step; // bottom right

    // get xyz
    float val[3] = {std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()};
    for (int i = 0; i < 3; ++i) {
      uint8_t tl_bytes[4] =
        {depth_.data[tl++], depth_.data[tl++], depth_.data[tl++], depth_.data[tl++]};
      float tl_float;
      std::memcpy(&tl_float, &tl_bytes, 4);

      if (std::isnan(tl_float)) {
        tr += 4; bl += 4; br += 4;
        continue;
      }

      uint8_t tr_bytes[4] =
        {depth_.data[tr++], depth_.data[tr++], depth_.data[tr++], depth_.data[tr++]};
      float tr_float;
      std::memcpy(&tr_float, &tr_bytes, 4);

      if (std::isnan(tr_float)) {
        bl += 4; br += 4;
        continue;
      }

      uint8_t bl_bytes[4] =
        {depth_.data[bl++], depth_.data[bl++], depth_.data[bl++], depth_.data[bl++]};
      float bl_float;
      std::memcpy(&bl_float, &bl_bytes, 4);

      if (std::isnan(bl_float)) {
        br += 4;
        continue;
      }

      uint8_t br_bytes[4] =
        {depth_.data[br++], depth_.data[br++], depth_.data[br++], depth_.data[br++]};
      float br_float;
      std::memcpy(&br_float, &br_bytes, 4);

      if (std::isnan(br_float))
        continue;

      val[i] = (tl_float + tr_float + bl_float + br_float) * 0.25;
    }

    auto x = reinterpret_cast<uint8_t*>(&val[0]);
    auto y = reinterpret_cast<uint8_t*>(&val[1]);
    auto z = reinterpret_cast<uint8_t*>(&val[2]);

    res.data[j++] = x[0]; res.data[j++] = x[1]; res.data[j++] = x[2]; res.data[j++] = x[3];
    res.data[j++] = y[0]; res.data[j++] = y[1]; res.data[j++] = y[2]; res.data[j++] = y[3];
    res.data[j++] = z[0]; res.data[j++] = z[1]; res.data[j++] = z[2]; res.data[j++] = z[3];

    // skip 4 bytes, rgb offset is 16 in original msg
    tl += 4; tr += 4; bl += 4; br += 4;

    // get rgb
    int rgb[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i)
      rgb[i] = static_cast<int>((static_cast<int>(depth_.data[tl++])
                                 + static_cast<int>(depth_.data[tr++])
                                 + static_cast<int>(depth_.data[bl++])
                                 + static_cast<int>(depth_.data[br++])) * 0.25);

    res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[0])[0];
    res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[1])[0];
    res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[2])[0];
    res.data[j++] = 0;

    at += stride_x;
    if (at - row * depth_.row_step > depth_.row_step - stride_x) {
      row += stride_y;
      at = row * depth_.row_step;
    }
  }
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
sensor_msgs::PointCloud2 XtionInterface::ReadPointsAfter(float _scale_x, float _scale_y)
{
  ros::Time time;
  for (int i=0; i < 100; ++i) {
    depth_mutex_.lock();
    time = depth_.header.stamp;
    depth_mutex_.unlock();
    if (time > time_now_) continue;
    usleep(100*1000);
  }
  return ReadPoints(_scale_x, _scale_y);
}

//////////////////////////////////////////////////
sensor_msgs::Image XtionInterface::ReadImageAfter()
{
  ros::Time time;
  for (int i=0; i < 100; ++i) {
    image_mutex_.lock();
    time = image_.header.stamp;
    image_mutex_.unlock();
    if (time > time_now_) continue;
    usleep(100*1000);
  }
  return ReadImage();
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
std::vector<sensor_msgs::RegionOfInterest> XtionInterface::ImageBounds
(std::vector<std::array<int, 4> > _depth_indicies, float _w_scale, float _h_scale)
{
  std::vector<std::array<int, 4> > rescaled_indices;
  rescaled_indices.reserve(_depth_indicies.size());

  int points_width = static_cast<int>(depth_width_ * _w_scale);
  int points_height = static_cast<int>(depth_height_ * _h_scale);
  int stride_x = static_cast<int>(1.0 / _w_scale);
  int stride_y = static_cast<int>(1.0 / _h_scale);

  for (auto it = _depth_indicies.begin(); it != _depth_indicies.end(); ++it) {
    std::array<int ,4> arr;
    for (int i = 0; i < 4; ++i) {
      int y = it->at(i) / points_width;
      arr.at(i) = y * stride_y * depth_width_ + (it->at(i) - y * points_width) * stride_x;
    }
    rescaled_indices.push_back(arr);
  }

  return ImageBounds(rescaled_indices);
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
void XtionInterface::SetNow()
{
  time_now_ = ros::Time::now();
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
