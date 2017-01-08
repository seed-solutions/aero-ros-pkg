#include "aero_std/aerocv.hh"
#include "aero_std/time.h"

//////////////////////////////////////////////////
std::pair<std::vector<aero::aerocv::objectarea>,
          std::vector<aero::aerocv::objectarea> > aero::aerocv::DetectObjectnessArea
(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud,
 cv::Mat &_img, cv::Vec3b _env_color, std::string _debug_folder)
{
  auto begin = aero::time::now();

  std::vector<aero::aerocv::objectarea> scene;
  std::vector<aero::aerocv::objectarea> env;
  int w_scale = _img.cols / _cloud->width;
  int h_scale = _img.rows / _cloud->height;

  // get normal
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(_cloud);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree
    (new pcl::search::KdTree<pcl::PointXYZRGB>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*normals);

  // cluster with region growing
  pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
  reg.setMinClusterSize(100);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(10);
  reg.setInputCloud(_cloud);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);
  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  // evaluate whether cluster is within expected range
  // this operation removes floors and walls
  float radius_threshold = 1.2; // meter
  for (auto it = clusters.begin(); it != clusters.end(); ) {
    Eigen::Vector3f center = {0.0, 0.0, 0.0};
    Eigen::Vector3f normal = {0.0, 0.0, 0.0};

    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      center +=
        Eigen::Vector3f(_cloud->points[*pit].x, _cloud->points[*pit].y,
                        _cloud->points[*pit].z);
      normal +=
        Eigen::Vector3f(normals->points[*pit].normal_x,
                        normals->points[*pit].normal_y,
                        normals->points[*pit].normal_z);
    }

    center /= it->indices.size();
    normal /= it->indices.size();
    if (center.norm() > radius_threshold) {
      it = clusters.erase(it);
      continue;
    }

    // add object to scene
    aero::aerocv::objectarea obj;
    obj.indices3d.assign(it->indices.begin(), it->indices.end());
    obj.visible3d = true;
    obj.center3d = center;
    normal.normalize();
    obj.normal3d = normal;
    scene.push_back(obj);
    ++it;
  }

  // find indices without a cluster label
  std::vector<uchar> indices_without_label(_cloud->points.size(), 255);
  for (auto it = clusters.begin(); it != clusters.end(); ++it)
    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      indices_without_label[*pit] = 0;

  // binary cluster non-labeled regions
  cv::Mat binary_img(_cloud->height, _cloud->width, CV_8U);
  int at = 0;
  for (unsigned int i = 0; i < binary_img.rows; ++i)
    for (unsigned int j = 0; j < binary_img.cols; ++j)
      binary_img.at<uchar>(i, j) = indices_without_label[at++];
  cv::Mat labeled_image;
  cv::Mat stats;
  cv::Mat centroids;
  int n_labels =
    cv::connectedComponentsWithStats(binary_img, labeled_image, stats, centroids);

  // analyze non-labeled regions
  int noise_threshold = 50; // pixels
  int outmost_label = -1;
  int max_outmost_area = 0;
  for (int k = 1; k < n_labels; ++k) {
    int *param = stats.ptr<int>(k);
    int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
    int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
    int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
    int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

    // remove noise
    if (width * height < noise_threshold) continue;
    // remove region adjacent to the edge of image
    // this removes floor, wall, and edge NaN noises all together
    // but will also remove some connected regions as well
    if (x == 0 || (x + width) == _cloud->width ||
        y == 0 || (y + height) == _cloud->height) {
      if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA] > max_outmost_area) {
        outmost_label = k; // keep track of largest edge region
        max_outmost_area = param[cv::ConnectedComponentsTypes::CC_STAT_AREA];
      }
      continue;
    }

    // add object to scene
    aero::aerocv::objectarea obj;
    obj.indices3d.reserve(param[cv::ConnectedComponentsTypes::CC_STAT_AREA]);
    for (unsigned int i = 0; i < height; ++i)
      for (unsigned int j = 0; j < width; ++j)
        if (labeled_image.at<int>(y + i, x + j) == k)
          obj.indices3d.push_back
            (static_cast<int>((y + i) * labeled_image.cols + x + j));
    obj.bounds2d =
      cv::Rect(x*w_scale, y*h_scale, width*w_scale, height*h_scale);
    obj.visible3d = false;
    scene.push_back(obj);
  }

  // largest edge region usually has a complex over connection
  // break region apart by suppressing countour regions
  // why this works: connections are usually due to undefined object edges
  // the suppression eliminates such undefined edges
  cv::Mat outmost(_cloud->height, _cloud->width, CV_8U);
  if (outmost_label > 0) {
    int at = 0;
    for (unsigned int i = 0; i < labeled_image.rows; ++i)
      for (unsigned int j = 0; j < labeled_image.cols; ++j)
        if (labeled_image.at<int>(i, j) == outmost_label)
          outmost.at<uchar>(i, j) = 255;
        else
          outmost.at<uchar>(i, j) = 0;

    // find contours destroyes image, so clone
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(outmost.clone(), contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    // break region apart by suppressing contours
    int suppress_margin = 6;
    for (auto contour = contours.begin(); contour != contours.end(); ++contour)
      cv::polylines(outmost, *contour, true, cv::Scalar(0), suppress_margin);

    // get new clusters
    cv::Mat labeled_image;
    cv::Mat stats;
    cv::Mat centroids;
    int n_labels =
      cv::connectedComponentsWithStats(outmost, labeled_image, stats, centroids, 4);

    // add clusters to list if not too small
    // because, clusters were suppressed, use a lower noise threshold
    // the upper threshold will remove backgrounds
    int lower_noise_threshold = 30;
    int upper_noise_threshold = 300;
    for (int k = 1; k < n_labels; ++k) {
      int *param = stats.ptr<int>(k);
      int x = param[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
      int y = param[cv::ConnectedComponentsTypes::CC_STAT_TOP];
      int height = param[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
      int width = param[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

      // remove noise
      if (width * height < lower_noise_threshold) continue;

      // big clusters nearby camera are likely to be objects, so do not remove
      // when the bound of background clusters covers the whole image,
      // the cluster center is the center of image
      // the distance threshold is set so that such clusters are removed
      if (y + 0.5 * height <= 0.5 * _cloud->height + 1)
        if (param[cv::ConnectedComponentsTypes::CC_STAT_AREA]
            > upper_noise_threshold)
          continue;

      aero::aerocv::objectarea obj;
      obj.indices3d.reserve(param[cv::ConnectedComponentsTypes::CC_STAT_AREA]);

      // get current cluster to check further division
      cv::Mat blob = cv::Mat::zeros(height, width, CV_8U);
      for (unsigned int i = 0; i < height; ++i)
        for (unsigned int j = 0; j < width; ++j)
          if (labeled_image.at<int>(y + i, x + j) == k) {
            blob.at<uchar>(i, j) = 255;
            obj.indices3d.push_back
              (static_cast<int>((y + i) * labeled_image.cols + x + j));
          }

      // conduct further division
      cv::Mat divs;
      cv::Mat divstats;
      cv::Mat divcentroids;
      int n_div =
        aero::aerocv::cutComponentsWithStats(
            blob, divs, divstats, divcentroids, _debug_folder + "objectness_");

      // if no further division
      if (n_div == 1) {
        obj.bounds2d =
          cv::Rect(x*w_scale, y*h_scale, width*w_scale, height*h_scale);
        obj.visible3d = false;
        scene.push_back(obj);
        continue;
      }

      // if there was further division
      for (int l = 1; l < n_div; ++l) {
        int *bounds = divstats.ptr<int>(l);
        int dx = bounds[cv::ConnectedComponentsTypes::CC_STAT_LEFT];
        int dy = bounds[cv::ConnectedComponentsTypes::CC_STAT_TOP];
        int dheight = bounds[cv::ConnectedComponentsTypes::CC_STAT_HEIGHT];
        int dwidth = bounds[cv::ConnectedComponentsTypes::CC_STAT_WIDTH];

        aero::aerocv::objectarea dobj;
        dobj.indices3d.reserve(bounds[cv::ConnectedComponentsTypes::CC_STAT_AREA]);
        for (unsigned int i = 0; i < dheight; ++i)
          for (unsigned int j = 0; j < dwidth; ++j)
            if (divs.at<int>(dy + i, dx + j) == k)
              dobj.indices3d.push_back
                (static_cast<int>((y + dy + i) * labeled_image.cols + x + dx + j));
        dobj.bounds2d =
          cv::Rect((x+dx)*w_scale, (y+dy)*h_scale, dwidth*w_scale, dheight*h_scale);
        dobj.visible3d = false;
        scene.push_back(dobj);
      }
    }
  }

  // analyze bounds, corners, color properties of RGS regions
  for (auto it = clusters.begin(); it != clusters.end(); ) {
    auto obj = scene.begin() + static_cast<int>(it - clusters.begin());

    std::vector<uchar> cluster1d(_cloud->points.size(), 0);
    std::vector<cv::Vec3b> colors1d(it->indices.size());
    int color1d_idx = 0;
    for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
      cluster1d[*pit] = 255;
      // in the meantime, get color as well
      uint32_t rgb = *reinterpret_cast<int*>(&_cloud->points[*pit].rgb);
      colors1d[color1d_idx++] = cv::Vec3b
        ((rgb) & 0x0000ff, (rgb >> 8) & 0x0000ff, (rgb >> 16) & 0x0000ff);
    }

    // get color information of cluster
    int quantize_amount = 4;
    std::vector<cv::Vec3b> dominant_colors(quantize_amount);
    aero::aerocv::medianCut(colors1d, 0, 0, dominant_colors);
    // check how many colors match environment color information
    float distance_threshold = 20;
    int expected_matches = 3;
    int matches = 0;
    for (auto c = dominant_colors.begin(); c != dominant_colors.end(); ++c)
      if (aero::aerocv::distance(aero::aerocv::rgb2lab(*c),
                                 aero::aerocv::rgb2lab(_env_color))
          < distance_threshold)
        ++matches;
    if (matches >= expected_matches) { // if likely environment
      env.push_back(*obj);
      it = clusters.erase(it);
      scene.erase(obj); // remove environment object(e.g. table) from scene
      if (_debug_folder != "")
        aero::aerocv::drawPalette
          (dominant_colors, _env_color, _debug_folder + "objectness_");
      continue;
    }

    // from color palette, get best matched color names
    aero::aerocv::getColorNames
      (dominant_colors, obj->properties.colors, aero::aerocv::colorMap9);

    // create cv::Mat from cluster1d
    cv::Mat cluster(_cloud->height, _cloud->width, CV_8U);
    int at = 0;
    for (unsigned int i = 0; i < cluster.rows; ++i)
      for (unsigned int j = 0; j < cluster.cols; ++j)
        cluster.at<uchar>(i, j) = cluster1d[at++];

    // get bounding box of cluster
    auto bb = cv::boundingRect(cluster);
    obj->bounds2d =
      cv::Rect(bb.x*w_scale, bb.y*h_scale, bb.width*w_scale, bb.height*h_scale);

    ++it;
  }

  auto end = aero::time::now();

  std::cout << "detection time " << aero::time::ms(end - begin) << std::endl;

  // view results if debug mode is true

  if (_debug_folder != "") {
    // draw bounds2d

    for (auto it = scene.begin(); it < scene.begin() + clusters.size(); ++it) {
      cv::rectangle(_img, it->bounds2d, cv::Scalar(0, 255, 0), 2);

      // write color properties info
      std::string text = "";
      for (auto c = it->properties.colors.begin();
           c != it->properties.colors.end(); ++c) {
        std::ostringstream val;
        val << std::setprecision(2) << c->second;
        text += c->first + "(" + val.str() + ") ";
      }
      cv::putText(_img, text,
                  cv::Point(it->bounds2d.x, it->bounds2d.y + it->bounds2d.height),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1.0);

      // draw norm
      float s = 10; // scale
      cv::Point center(it->bounds2d.x + 0.5 * it->bounds2d.width,
                       it->bounds2d.y + 0.5 * it->bounds2d.height);
      // note: normal z value is likely negative
      cv::Point normal(
          s * static_cast<float>(it->normal3d.x()) / fabs(it->normal3d.z()),
          s * static_cast<float>(it->normal3d.y()) / fabs(it->normal3d.z()));
      cv::line(_img, center, cv::Point(center.x + normal.x, center.y + normal.y),
               cv::Scalar(255, 255, 0));
      cv::circle(_img, cv::Point(center.x + normal.x, center.y + normal.y),
                 2, cv::Scalar(255, 255, 0), 2);
    }

    for (auto it = scene.begin() + clusters.size(); it != scene.end(); ++it)
      cv::rectangle(_img, it->bounds2d, cv::Scalar(255, 0, 0), 2);

    // show results

    cv::imwrite(_debug_folder + "objectness_mid1.jpg", binary_img);
    if (outmost_label > 0)
      cv::imwrite(_debug_folder + "objectness_mid2.jpg", outmost);
    cv::imwrite(_debug_folder + "objectness.jpg", _img);
  }

  return {scene, env};
}

//////////////////////////////////////////////////
std::vector<int> aero::aerocv::FindTarget
(std::string _target_color, std::vector<aero::aerocv::objectarea> &_scene)
{
  std::vector<std::pair<int, float> > candidates;

  for (auto obj = _scene.begin(); obj != _scene.end(); ++obj) {
    if (!obj->visible3d) break;
    for (auto c = obj->properties.colors.begin();
         c != obj->properties.colors.end(); ++c)
      if (c->first == _target_color) {
        candidates.push_back({static_cast<int>(obj - _scene.begin()), c->second});
        break;
      }
  }

  if (candidates.size() == 0)
    return std::vector<int> {};

  // order candidates such that best match comes first
  std::sort(candidates.begin(), candidates.end(),
            [](std::pair<int, float> x, std::pair<int, float> y) -> bool {
              return (x.second > y.second);
            });

  // divide candidates into fields depending on color score
  std::vector<std::vector<std::pair<int, float> > > ordered_candidates(1);
  auto oc = ordered_candidates.begin();
  float best_score = candidates.begin()->second;
  for (auto obj = candidates.begin(); obj != candidates.end(); ++obj) {
    auto s = _scene.begin() + obj->first;
    if (fabs(obj->second - best_score) < 0.1) {
      oc->push_back({obj->first, s->center3d.norm()});
    } else { // any score with more than 0.1 difference, add to next field
      best_score = obj->second;
      ordered_candidates.push_back({{obj->first, s->center3d.norm()}});
      oc = ordered_candidates.end() - 1;
    }
  }

  // sort matches by distance and add to result
  std::vector<int> result(candidates.size());
  auto it = result.begin();
  for (auto c = ordered_candidates.begin(); c != ordered_candidates.end(); ++c) {
    std::sort(c->begin(), c->end(),
              [](std::pair<int, float> x, std::pair<int, float> y) -> bool {
                return (x.second < y.second);
              });
    for (auto obj = c->begin(); obj != c->end(); ++obj)
      *it++ = obj->first;
  }

  return result;
}

//////////////////////////////////////////////////
aero::aerocv::graspconfig aero::aerocv::ConfigurationFromLocal1DState
(int _target, std::vector<aero::aerocv::objectarea> &_scene,
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud, cv::Mat &_img,
 std::string _debug_folder)
{
  aero::aerocv::graspconfig result;
  auto target = _scene.begin() + _target;

  // check sparsity of surroundings
  // here, objects with more than two detected facets will not be sparse
  // only objects with one detected facet and are far apart will be sparse
  float distance_threshold = 0.15; // m
  std::vector<std::vector<aero::aerocv::objectarea>::iterator > nearby_objects;
  for (auto obj = _scene.begin(); obj != _scene.end(); ++obj) {
    if (!obj->visible3d) continue; // currently not supported
    if (obj == target) continue; // obviously, don't include self
    if ((obj->center3d - target->center3d).norm() < distance_threshold)
      nearby_objects.push_back(obj);
  }

  if (nearby_objects.size() == 0) {
    std::cout << "sparse top-grasp-able!\n";
    result.sparse = 1;
    result.contact = 0;
    return result;
  }

  // get indices bounds of target object
  // note, bounds2d is image coords so cannot be used as comparison
  int bound_y = std::numeric_limits<int>::max();
  int bound_left = std::numeric_limits<int>::max();
  int bound_right = -1;
  for (auto p = target->indices3d.begin(); p != target->indices3d.end(); ++p) {
    int y = *p / _cloud->width;
    int x = *p - y * _cloud->width;
    if (y < bound_y) bound_y = y;
    if (x < bound_left) bound_left = x;
    if (x > bound_right) bound_right = x;
  }

  // check top grasp-ability from environment
  // check depth of points behind target
  float depth_threshold = 0.02; // m
  std::vector<std::vector<aero::aerocv::objectarea>::iterator > objects_behind;
  for (auto it = nearby_objects.begin(); it != nearby_objects.end(); ++it) {
    auto obj = *it;

    for (auto p = obj->indices3d.begin(); p != obj->indices3d.end(); ++p) {
      int y = *p / _cloud->width;
      int x = *p - y * _cloud->width;
      // check if point is behind object
      if (y < bound_y && bound_left < x && x < bound_right)
        // check if point is deeper than target
        if (_cloud->points[*p].z < target->center3d.z() + depth_threshold) {
          objects_behind.push_back(obj);
          break;
        }
    }
  }

  // if target is not buried, target is likely top-grasp-able
  if (objects_behind.size() == 0) {
    std::cout << "not buried top-grasp-able!\n";
    result.sparse = 0;
    result.contact = 0;
    return result;
  }

  // check norm of objects behind and check for facets
  bool detected_parallel = false;
  std::vector<std::vector<aero::aerocv::objectarea>::iterator > facets;
  for (auto it = objects_behind.begin(); it != objects_behind.end(); ++it) {
    auto obj = *it;

    // find objects right behind considering orientation
    // the following projected dot product can calculate this
    Eigen::Vector3f v = target->center3d - obj->center3d;
    float c_norm = std::sqrt(std::pow(v.x(), 2) + std::pow(v.y(), 2));
    float n_norm =
      std::sqrt(std::pow(target->normal3d.x(), 2)
                + std::pow(target->normal3d.y(), 2));
    if (fabs(v.x() * target->normal3d.x() + v.y() * target->normal3d.y())
        < 0.7 * c_norm * n_norm)
      continue;

    // check clockwise-ness of 3d dot product
    // parallel > front facing > slightly bigger object behind (contact-grasp-able)
    // c clockwise > front facing > detected facet behind (further analyze)
    // clockwise > top facing > detected object preventing grasp (not grasp-able)
    float dot3d = target->normal3d.dot(obj->normal3d);

    if (0.7 < dot3d) { // likely parallel
      detected_parallel = true;
      continue;
    }

    if (fabs(dot3d) < 0.3) { // likely perpendicular
      Eigen::Vector3f crossv = target->normal3d.cross(obj->normal3d);
      crossv.normalize();
      float det3d = Eigen::Vector3f(-1, 0, 0).dot(crossv);
      if (det3d < 0) { // likely clockwise (higher priority than parallel)
        std::cout << "buried not grasp-able!\n";
        result.sparse = 0;
        result.contact = -1;
        // report what object may be preventing grasp
        result.facets.push_back(static_cast<int>(obj - _scene.begin()));
        return result;
      }
      // likely counter clockwise
      facets.push_back(obj);
    }
  }

  // at least not buried
  if (detected_parallel) {
    std::cout << "large object behind, insist contact grasp!\n";
    result.sparse = 0;
    result.contact = 1;
    return result;
  }

  // if no objects behind are preventing grasp
  if (facets.size() == 0) {
    std::cout << "no direct behind objects found, likely top-grasp-able!\n";
    result.sparse = 0;
    result.contact = 0;
    return result;
  }

  // analyze whether facet is pile of objects or single facets
  for (auto it = facets.begin(); it != facets.end(); ++it) {
    auto obj = *it;

    // get facet image as gray
    cv::Mat gray;
    cv::cvtColor(_img(obj->bounds2d), gray, CV_RGB2GRAY);
    cv::Mat stats; // information of each package cluster if found
    if (aero::aerocv::findPackagePattern(gray, stats, _debug_folder)) {
      std::cout << "detected pile, likely contact grasp-able!\n";
      result.sparse = 0;
      result.contact = 1;
      return result;
    }

    // keep track of facet id
    result.facets.push_back(static_cast<int>(obj - _scene.begin()));
  }

  // code exits here
  // recursively call ConfigurationFromLocal1DState when facet is detected
  std::cout << "detected facet, state could not be determined!\n";
  result.sparse = -1;
  result.contact = -1;
  return result;
}
