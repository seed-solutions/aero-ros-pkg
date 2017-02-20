#include "ObjectFeatures.hh"

using namespace aero;
using namespace vision;

//////////////////////////////////////////////////
ObjectFeatures::ObjectFeatures(ros::NodeHandle _nh)
  : nh_(_nh), pseudo_tf_spinner_(1, &pseudo_tf_queue_)
{
  base_to_eye_.position.x = 0;
  base_to_eye_.position.y = 0;
  base_to_eye_.position.z = 0;
  base_to_eye_.orientation.x = 1;
  base_to_eye_.orientation.y = 0;
  base_to_eye_.orientation.z = 0;
  base_to_eye_.orientation.w = 0;

  ros::SubscribeOptions pseudo_tf_ops =
    ros::SubscribeOptions::create<geometry_msgs::Pose>
    ("/matrix/base_to_eye",
     100,
     boost::bind(&ObjectFeatures::SubscribeCameraPseudoTf, this, _1),
     ros::VoidPtr(),
     &pseudo_tf_queue_);

  camera_pseudo_tf_subscriber_ = nh_.subscribe(pseudo_tf_ops);

  pseudo_tf_spinner_.start();

  pcl_pub_ =
    nh_.advertise<sensor_msgs::PointCloud2>("/object_features/points", 1000);

  img_pub_ =
    nh_.advertise<sensor_msgs::Image>("/object_features/image", 1000);
}

//////////////////////////////////////////////////
ObjectFeatures::~ObjectFeatures()
{
}

//////////////////////////////////////////////////
Eigen::Vector3f ObjectFeatures::ConvertWorld(Eigen::Vector3f _pos_camera)
{
  Eigen::Quaternionf base_to_eye_q =
    Eigen::Quaternionf(base_to_eye_.orientation.x,
		       base_to_eye_.orientation.y,
		       base_to_eye_.orientation.z,
		       base_to_eye_.orientation.w);

  return
    Eigen::Vector3f(base_to_eye_.position.x,
		    base_to_eye_.position.y,
		    base_to_eye_.position.z) + base_to_eye_q * _pos_camera;
}

//////////////////////////////////////////////////
void ObjectFeatures::Convert_mm(geometry_msgs::Point& _pos)
{
  _pos.x = 1000 * _pos.x;
  _pos.y = 1000 * _pos.y;
  _pos.z = 1000 * _pos.z;
}

//////////////////////////////////////////////////
// void ObjectFeatures::BroadcastTf(Eigen::Vector3f _position, std::string _name, std::string _frame)
// {
//   BroadcastTf(_position, Eigen::Quaternionf(1, 0, 0, 0), _name, _frame);
// }

//////////////////////////////////////////////////
// void ObjectFeatures::BroadcastTf(Eigen::Vector3f _position,
//                                  Eigen::Quaternionf _orientation,
//                                  std::string _name,
//                                  std::string _frame)
// {
//   tf_mutex_.lock();
//   broadcast_tf_ = true;
//   tf_mutex_.unlock();

//   std::thread newthread
//     ([&](Eigen::Vector3f position,
//          Eigen::Quaternionf orientation,
//          std::string name,
//          std::string frame) {
//       bool broadcast_flag = true;
//       while (broadcast_flag) {
//         static tf::TransformBroadcaster br;
//         tf::Transform transform;
//         transform.setOrigin(tf::Vector3(position.x(),
//                                         position.y(),
//                                         position.z()));
//         tf::Quaternion q(orientation.x(),
//                          orientation.y(),
//                          orientation.z(),
//                          orientation.w());
//         transform.setRotation(q);
//         br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
//                                               _frame, name));
//         tf_mutex_.lock();
//         broadcast_flag = broadcast_tf_;
//         tf_mutex_.unlock();
//       }
//       usleep(100 * 1000);
//     }, _position, _orientation, _name, _frame);
//   newthread.detach();
// }


// }
//////////////////////////////////////////////////
void ObjectFeatures::BroadcastTf(Eigen::Vector3f _position, std::string _name)
{
  BroadcastTf(_position, Eigen::Quaternionf(1, 0, 0, 0), _name);
}

void ObjectFeatures::BroadcastTf(Eigen::Vector3f _position,
                                 Eigen::Quaternionf _orientation,
                                 std::string _name)
{
  tf_mutex_.lock();
  broadcast_tf_ = true;
  tf_mutex_.unlock();

  std::thread newthread
    ([&](Eigen::Vector3f position,
         Eigen::Quaternionf orientation,
         std::string name) {
      bool broadcast_flag = true;
      while (broadcast_flag) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(position.x(),
                                        position.y(),
                                        position.z()));
        tf::Quaternion q(orientation.x(),
                         orientation.y(),
                         orientation.z(),
                         orientation.w());
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                             "base_link" ,name));
        tf_mutex_.lock();
        broadcast_flag = broadcast_tf_;
        tf_mutex_.unlock();
      }
      usleep(100 * 1000);
    }, _position, _orientation, _name);
  newthread.detach();
}

//////////////////////////////////////////////////
void ObjectFeatures::StopBroadcastTfAll()
{
  tf_mutex_.lock();
  broadcast_tf_ = false;
  tf_mutex_.unlock();
}

#ifdef _CFG_DEPENDS_PCL_
//////////////////////////////////////////////////
void ObjectFeatures::BroadcastPoints
(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _points, std::string _frame)
{
  pcl::PCLPointCloud2 pcl_out;
  sensor_msgs::PointCloud2 msg;
  pcl::toPCLPointCloud2(*_points, pcl_out);
  pcl_conversions::fromPCL(pcl_out, msg);
  msg.header.frame_id = _frame;
  msg.header.stamp = ros::Time(0); // get possible recent
  pcl_pub_.publish(msg);
}
#endif

#ifdef _CFG_DEPENDS_OPENCV_
//////////////////////////////////////////////////
void ObjectFeatures::BroadcastImage(cv::Mat _img, std::string _frame)
{
  std::vector<uint8_t> data;
  data.reserve(3 * 1920 * 1080);
  int pixel_count = 0;
  for (auto it = _img.begin<cv::Vec3b>(); it != _img.end<cv::Vec3b>(); ++it) {
    data.push_back((*it)[0]);
    data.push_back((*it)[1]);
    data.push_back((*it)[2]);
    ++pixel_count;
  }
  data.resize(3 * pixel_count);

  sensor_msgs::Image msg;
  msg.header.frame_id = _frame;
  msg.header.stamp = ros::Time(0);
  msg.height = _img.rows;
  msg.width = _img.cols;
  msg.step = 3 * msg.width;
  msg.encoding = "bgr8";
  msg.is_bigendian = true;
  msg.data.assign(data.begin(), data.end());

  img_pub_.publish(msg);
}

//////////////////////////////////////////////////
cv::Mat ObjectFeatures::ResizeImage(cv::Mat _img, int _width, int _height)
{
  cv::Mat result = cv::Mat_<cv::Vec3b>(_height, _width, cv::Vec3b(255, 255, 255));

  float scale;
  cv::Rect roi;
  if (_img.cols >= _img.rows) {
    scale = static_cast<float>(_width) / _img.cols;
    roi.x = 0;
    roi.y = 0;
    roi.width = _width;
    roi.height = _img.rows * scale;
  } else {
    scale = static_cast<float>(_height) / _img.rows;
    roi.x = 0;
    roi.y = 0;
    roi.width = _img.cols * scale;
    roi.height = _height;
  }

  cv::resize(_img, result(roi), roi.size());

  return result;
}
#endif

//////////////////////////////////////////////////
void ObjectFeatures::SubscribeCameraPseudoTf
(const geometry_msgs::Pose::ConstPtr& _pose)
{
  base_to_eye_.position.x = _pose->position.x;
  base_to_eye_.position.y = _pose->position.y;
  base_to_eye_.position.z = _pose->position.z;
  base_to_eye_.orientation.x = _pose->orientation.x;
  base_to_eye_.orientation.y = _pose->orientation.y;
  base_to_eye_.orientation.z = _pose->orientation.z;
  base_to_eye_.orientation.w = _pose->orientation.w;
}
