#include "aero_addon/GraspAddonExtra.hh"

using namespace aero;
using namespace addon;

GraspAddonExtra::GraspAddonExtra
(ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _robot,
 aero::vision::ObjectFeaturesPtr _features,
 aero::addon::RotateKinectAddonPtr _kcon,
 kinect::interface::KinectInterfacePtr _kinect)
  : GraspAddon(_nh, _robot)
{
  features_ = _features;

  kcon_ = _kcon;

  kinect_ = _kinect;

  resize_x_ = 0.25;

  resize_y_ = 0.334;
}

GraspAddonExtra::~GraspAddonExtra()
{
}

bool GraspAddonExtra::sendBoxTumbleInitialLifter(Eigen::Vector3d _target)
{
  // send initial pose for recognition
  float default_base_height = 0.58;
  tmb_initial_lifter_z_ = _target.z() - default_base_height - 0.2;
  if (!robot_->sendLifter(0.0, tmb_initial_lifter_z_, 2000)) {
    std::string errmsg = "In find tumble: bad initial lifter move.";
    robot_->speakAsync(errmsg);
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }
  usleep(1000 * 1000); // make sure lifter move has finished
  return true;
}

bool GraspAddonExtra::findBoxTumbleParameters
(Eigen::Vector3d _target, float _width, float _height, cv::Rect _bounds2d,
 std::string _file, std::string _dbgfolder, tumbleparameters _params)
{
  tmb_av_.clear();
  tmb_reach_av_.clear();
  tmb_lifter_av_.clear();

  robot_->openHand(_params.arm);

  // find roi seed
  float roi_left = _target.y() - 0.2 * _width;
  float roi_right = _target.y() - 0.45 * _width;
  float roi_above = _target.z() + 0.45 * _height;
  float roi_below = _target.z() - 0.45 * _height;

  int seed_j = _bounds2d.y;
  int seed_i = static_cast<int>(_bounds2d.x + _bounds2d.width * 0.75);
  int seed_depth_index =
    seed_j * kinect_->ReadPoints(resize_x_, resize_y_).width + seed_i;

  // set target goal
  Eigen::Vector3f target_goal
    (_target.x(), _target.y() + _width * 0.5, _target.z());

  printf("target: %f, %f, %f\n", target_goal.x(), target_goal.y(), target_goal.z());
  printf("(a:%f, b:%f, l:%f, r:%f)\n", roi_above, roi_below, roi_left, roi_right);
  printf("seed: (%d, %d)\n", seed_j, seed_i);

  float reference_depth =
    tmbGetDepthOfTarget(seed_depth_index,
                        roi_above, roi_below, roi_left, roi_right,
                        tmb_initial_lifter_z_, _dbgfolder, 0);

  // bad depth
  if (reference_depth < 0)
    return false;

  printf("depth of target: %f\n", reference_depth);

  // get initial pose
  if (!tmbInitiateBoxTumble(_params.arm, target_goal, _params.d, _params.omega))
    return false;
  if (!tmbAdjustInitialHandHeight(_params.arm,
                                  _target.z(), _height, _params.level_margin))
    return false;

  // go to initial pose
  robot_->sendAngleVector(tmb_av_.at(0), 2000, aero::ikrange::lifter);
  robot_->sendAngleVector(tmb_av_.at(1), 2000, aero::ikrange::lifter);

  robot_->setRobotStateToCurrentState();

  // check if hand can be reached forward
  if (!tmbReachHandForward(_params.arm, _target.x(), 0.1))
    return false;

  // // go to initial pose
  // robot_->sendAngleVector(tmb_av_.at(0), 2000, aero::ikrange::lifter);
  // robot_->sendAngleVector(tmb_av_.at(1), 2000, aero::ikrange::lifter);
  // send lifter reach if required
  if (tmb_av_.size() > 2)
    robot_->sendAngleVector(tmb_av_.at(2), 1000, aero::ikrange::lifter);
  // reach hand forward
  int iter = _params.shortcut;
  int max_reach_iter = 10;
  auto traj = tmb_reach_av_.end() - max_reach_iter + iter;
  std::vector<std::map<aero::joint, double> >
    zeroth_traj(tmb_reach_av_.begin(),
                tmb_reach_av_.end() - max_reach_iter + iter + 1);
  robot_->sendTrajectory(zeroth_traj, 100 * zeroth_traj.size());

  robot_->setRobotStateToCurrentState();

  // save lifter position before tilt hand back
  std::map<aero::joint, double> pos0;
  robot_->getLifter(pos0);

  // start main iteration
  float angle_param;
  float press_param;
  float update_depth;
  bool success_flag = false;

  while (1) {
    bool finish = false; // escape from while flag
    for (int i = 1; i < 6; ++i) {
      press_param = 0.01 * i;
      angle_param = 0.349; // try tilting 20 degrees
      tmb_lifter_av_.clear();
      if (!tmbTiltHandBackward(angle_param, _height, _params.r, press_param))
        return false;
      robot_->sendLifterTrajectory(tmb_lifter_av_, 1000 * tmb_lifter_av_.size());
      update_depth = // get depth feedback
        tmbGetDepthOfTarget(seed_depth_index,
                            roi_above, roi_below, roi_left, roi_right,
                            tmb_initial_lifter_z_, _dbgfolder, iter*10+i);
      printf("with iter %d press param %f depth of target %f\n",
             iter, press_param, update_depth);

      // if item has tilted
      if (update_depth < reference_depth - _params.depth_thre) {
        // use slightly lower thre as item will be pushed back while tumbling
        float depth_thre_n = _params.depth_thre - 0.002;

        while (angle_param < 1.048) {
          std::map<aero::joint, double> pos1;
          robot_->getLifter(pos1);
          if (!robot_->sendLifter(pos1.at(aero::joint::lifter_x),
                  pos0.at(aero::joint::lifter_z) - press_param * 0.5, 1000)) {
            std::string errmsg = "In find tumble: release item failed.";
            robot_->speakAsync(errmsg);
            ROS_ERROR("%s", errmsg.c_str());
            return false;
          }
          usleep(1000 * 1000);
          update_depth = // get depth feedback
            tmbGetDepthOfTarget(seed_depth_index,
                                roi_above, roi_below, roi_left, roi_right,
                                tmb_initial_lifter_z_, _dbgfolder,
                                angle_param * 1000);
          printf("tilting %f depth of target %f\n", angle_param, update_depth);

          // if item has tumbled
          if (update_depth < reference_depth - depth_thre_n) { // in hand
            success_flag = true;
            break; // escape while (angle_param)
          } else if (update_depth > reference_depth + 0.01) { // out of hand
            break; // escape while (angle_param)
          }

          // reset state
          robot_->sendLifter(pos0.at(aero::joint::lifter_x),
                             pos0.at(aero::joint::lifter_z), 1000);
          robot_->setRobotStateVariables(*traj);
          robot_->sendAngleVector(500);
          usleep(500 * 1000);
          angle_param += 0.1; // add 5 degrees
          tmbTiltHandBackward(angle_param, _height, _params.r, press_param);
        }

        if (angle_param < 1.048) {
          finish = true; // answer was found within tumble try
          break; // escape for (press_param)
        }
      }

      // reset state
      robot_->sendLifter(pos0.at(aero::joint::lifter_x),
                         pos0.at(aero::joint::lifter_z), 1000);
    }

    if (finish) {
      break; // escape while loop
    } else { // try and reach hand forward           
      ++iter;
      if (iter >= max_reach_iter) {
        --iter;
        break;
      }
      ++traj;
      robot_->setRobotStateVariables(*traj);
      robot_->sendAngleVector(500);
      usleep(500 * 1000);
    }
  }

  if (!success_flag) { // failed tumble grasp
    std::string errmsg = "In find tumble: failed to find parameters.";
    robot_->speakAsync(errmsg);
    ROS_ERROR("%s", errmsg.c_str());
    return false;
  }

  tmb_av_.clear();
  tmb_reach_av_.clear();
  tmb_lifter_av_.clear();

  float reach_param = iter * 0.01;
  printf("reach param: %f\n", reach_param);
  printf("angle param: %f\n", angle_param);
  printf("press param: %f\n", press_param);

  // save parameters to file
  std::string save_filename = _file;
  if (_dbgfolder != "")
    save_filename = _dbgfolder + _file + ".yaml";
  std::ofstream ofs(save_filename);

  ofs << "item_id: " << 0 << std::endl; // not used
  // main parameters
  ofs << "reach_param: " << reach_param << std::endl;
  ofs << "press_param: " << press_param << std::endl;
  ofs << "angle_param: " << angle_param << std::endl;

  // settings for learning parameters
  ofs << "initial_lifter_x: " << 0.0 << std::endl; // not used
  ofs << "initial_lifter_z: " << 0.0 << std::endl; // not used
  ofs << "initial_hand_angle: " << _params.omega << std::endl;
  ofs << "level_margin: " << _params.level_margin << std::endl;
  ofs << "max_reach_iter: " << max_reach_iter << std::endl; // not used
  ofs << "patch_search_cols: " << 0 << std::endl; // not used
  ofs << "patch_include_depths: " << 0.0 << std::endl; // not used

  // vision results for reference
  ofs << "width: " << _width << std::endl; // not used
  ofs << "height: " << _height << std::endl; // not used
  ofs << "d: " << _params.d << std::endl;
  ofs << "initial_target_position: " << _target.x() << std::endl; // not used
  ofs << "initial_target_position: " << _target.y() << std::endl; // not used
  ofs << "initial_target_position: " << _target.z() << std::endl; // not used
  ofs.close();

  return true;
}

float GraspAddonExtra::tmbGetDepthOfTarget
(float _seed_depth_index, 
 float _roi_above, float _roi_below, float roi_left, float roi_right,
 float _initial_lifter_z, std::string _dbgfolder, int _savenum)
{
  auto points = kinect_->ReadPoints(resize_x_, resize_y_);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
    (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 pcl;
  pcl_conversions::toPCL(points, pcl);
  pcl::fromPCLPointCloud2(pcl, *cloud);

  std::map<aero::joint, double> pos;
  robot_->getLifter(pos);
  int stride;
  if (pos.at(aero::joint::lifter_z) > _initial_lifter_z)
    stride = cloud->width; // points have shifted downward
  else
    stride = -cloud->width; // points have shifted upward
  printf("stride: %d\n", stride);

  auto kinect_pose = kcon_->getBaseToEye();
  features_->setCameraTransform
    ("base_link", kinect_pose.first.cast<double>(), kinect_pose.second.cast<double>());

  // find likely target upper bound position
  int a_idx = _seed_depth_index;
  if (!tmbFindABBoundIdx(a_idx, _roi_above, stride, cloud)) {
    std::string errmsg = "In find tumble: lost target.";
    robot_->speakAsync(errmsg);
    ROS_ERROR("%s", errmsg.c_str());
    return -1; // lost target
  }
  printf("a_idx: %d\n", a_idx);

  // find likely target lower bound position
  int b_idx = a_idx + cloud->width;
  if (!tmbFindABBoundIdx(b_idx, _roi_below, cloud->width, cloud)) {
    std::string errmsg = "In find tumble: lost target.";
    robot_->speakAsync(errmsg);
    ROS_ERROR("%s", errmsg.c_str());
    return -1; // lost target
  }
  printf("b_idx: %d\n", b_idx);

  // find likely target left and right bound position
  int r_idx = b_idx, l_idx = b_idx; // b_idx is more stable than a_idx
  int row_start = static_cast<int>(b_idx / cloud->width) * cloud->width;
  int row_end = row_start + cloud->width;
  printf("search lr in idx: %d ~%d\n", row_start, row_end);
  if (!tmbFindLRBoundIdx(r_idx, row_start, row_end, roi_right, cloud)) {
    std::string errmsg = "In find tumble: lost target.";
    robot_->speakAsync(errmsg);
    ROS_ERROR("%s", errmsg.c_str());
    return -1; // lost target
  }
  if (!tmbFindLRBoundIdx(l_idx, row_start, row_end, roi_left, cloud)) {
    std::string errmsg = "In find tumble: lost target.";
    robot_->speakAsync(errmsg);
    ROS_ERROR("%s", errmsg.c_str());
    return -1; // lost target
  }
  printf("r_idx: %d\n", r_idx);
  printf("l_idx: %d\n", l_idx);

  int i0 = r_idx - row_start;
  int i1 = l_idx - row_start;
  int j0 = a_idx / cloud->width;
  int j1 = b_idx / cloud->width;
  printf("a~b: %d ~ %d, l~r: %d~%d\n", j0, j1, i1, i0);
  std::vector<float> depths_in_roi;
  float average_depth = 0.0;
  for (int j = j0; j <= j1; ++j)
    for (int i = i0; i <= i1; ++i) {
      auto p = cloud->points.at(j * cloud->width + i);
      // skip if value is invalid
      if (std::isnan(p.y) || std::isnan(p.x) || std::isnan(p.z))
        continue;
      depths_in_roi.push_back(p.z);
      average_depth += p.z;
    }

  if (depths_in_roi.size() == 0) {
    std::string errmsg = "In find tumble: roi is 0.";
    robot_->speakAsync(errmsg);
    ROS_ERROR("%s", errmsg.c_str());
    return -1; // lost target
  }

  average_depth /= depths_in_roi.size();
  average_depth =
    features_->convertWorld(Eigen::Vector3d(0, 0, average_depth)).x();

  // save roi image
  if (_dbgfolder != "") {
    cv::Mat dbgimg = cv::Mat::zeros(cloud->height, cloud->width, CV_8UC3);
    int depth_at = 0;
    for (unsigned int j = 0; j < cloud->height; ++j)
      for (unsigned int i = 0; i < cloud->width; ++i) {
        auto p = cloud->points.at(depth_at++);
        dbgimg.at<cv::Vec3b>(j, i) = cv::Vec3b(p.b, p.g, p.r);
      }
    cv::imwrite
      (_dbgfolder + "roi" + std::to_string(_savenum) + ".jpg",
       dbgimg(cv::Rect(i0, j0, i1-i0+1, j1-j0+1)));
    cv::rectangle
      (dbgimg, cv::Rect(i0, j0, i1-i0+1, j1-j0+1), cv::Scalar(255, 0, 255), 5);
    cv::imwrite
      (_dbgfolder + "roi_in_img" + std::to_string(_savenum) + ".jpg", dbgimg);
  }

  std::sort(depths_in_roi.begin(), depths_in_roi.end());
  float medium_depth = features_->convertWorld
    (Eigen::Vector3d(0, 0, depths_in_roi.at(depths_in_roi.size() >> 1))).x();
  printf("medium_depth: %f\n", medium_depth);
  printf("average_depth: %f\n", average_depth);

  // return average_depth;
  return medium_depth;
}

bool GraspAddonExtra::tmbFindABBoundIdx
(int &_idx, float _ref, float _stride,
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
{
  if (_stride > 0)
    while (_idx < _cloud->points.size()) {
      Eigen::Vector3d p(_cloud->points.at(_idx).x,
                        _cloud->points.at(_idx).y,
                        _cloud->points.at(_idx).z);
      float val = features_->convertWorld(p).z();
      // printf("%f\n", val);
      if (!std::isnan(_cloud->points.at(_idx).y)
          && (fabs(val - _ref) < 0.01 || val < _ref))
        break;
      _idx += _stride;
    }
  else
    while (_idx < _cloud->points.size()) {
      Eigen::Vector3d p(_cloud->points.at(_idx).x,
                        _cloud->points.at(_idx).y,
                        _cloud->points.at(_idx).z);
      float val = features_->convertWorld(p).z();
      // printf("%f\n", val);
      if (!std::isnan(_cloud->points.at(_idx).y)
          && (fabs(val - _ref) < 0.01 || val > _ref))
        break;
      _idx += _stride;
    }

  // if find failed
  if (_idx >= _cloud->points.size())
    return false;

  return true;
}

bool GraspAddonExtra::tmbFindLRBoundIdx
(int &_idx, int _row_start, int _row_end, float _ref,
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
{
  Eigen::Vector3d p(_cloud->points.at(_idx).x,
                    _cloud->points.at(_idx).y,
                    _cloud->points.at(_idx).z);
  if (features_->convertWorld(p).y() > _ref) {
    // printf("> ref\n", _ref);
    while (_idx < _row_end) {
      Eigen::Vector3d p(_cloud->points.at(_idx).x,
                        _cloud->points.at(_idx).y,
                        _cloud->points.at(_idx).z);
      float val = features_->convertWorld(p).y();
      // printf("%f\n", val);
      if (!std::isnan(_cloud->points.at(_idx).x)
          && (fabs(val - _ref) < 0.01 || val < _ref))
        break;
      _idx -= 1;
    }
    if (_idx >= _row_end)
      return false;
  } else {
    // printf("<= ref\n", _ref);
    while (_idx >= _row_start) {
      Eigen::Vector3d p(_cloud->points.at(_idx).x,
                        _cloud->points.at(_idx).y,
                        _cloud->points.at(_idx).z);
      float val = features_->convertWorld(p).y();
      // printf("%f\n", val);
      if (!std::isnan(_cloud->points.at(_idx).x)
          && (fabs(val - _ref) < 0.01 || val > _ref))
        break;
      _idx += 1;
    }
    if (_idx < _row_start)
      return false;
  }

  return true;
}
