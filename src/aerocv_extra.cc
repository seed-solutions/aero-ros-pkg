#include "aero_std/aerocv_extra.hh"

//////////////////////////////////////////////////
void ProjectImage(std::vector<aero::aerocv::objectarea> &_scene,
                  cv::Mat &_img, std::vector<cv::Mat> &projected_images,
                  std::string _debug_folder)
{
  auto img = projected_images.begin();
  for (auto obj = _scene.begin(); obj != _scene.end(); ++obj) {
    if (!obj->visible3d) {
      // currently no transformation for non-visible objects
      *img = _img(obj->bounds2d);
      ++img;
      continue;
    }

    // estimate whether object is standing or laying
    // assumes y positive downward, z positive forward
    float prob_standing = obj->normal3d.dot(Eigen::Vector3f(0, 0, -1));
    float prob_laying = obj->normal3d.dot(Eigen::Vector3f(0, -1, 0));
    bool is_standing = false;
    if (prob_standing > prob_laying + 0.1) // set to laying for ambiguous
      is_standing = true;

    // enlargen bounds if RGS image
    // usually standing RGS images do not capture the edge slope
    // when laying, there may be misplacement between 2D and 3D
    cv::Rect bounds = obj->bounds2d;
    int pad_top = 50; // pixels

    if (obj->bounds2d.y - pad_top > 0) {
      bounds.y = obj->bounds2d.y - pad_top;
      bounds.height += pad_top;
    } else {
      pad_top = obj->bounds2d.y;
      bounds.height += pad_top;
      bounds.y = 0;
    }

    cv::Mat raw = _img(bounds);

    // setup projection
    // the source corners uses a pre-defined transformation map
    // why not use fft? -> images are usually not cropped ideally
    cv::Point2f tl, tr, br, bl;
    int offset;
    int trans_y;
    if (is_standing) {
      int x = static_cast<int>((bounds.x + 0.5 * bounds.width) / _img.cols
          * aero::aerocv::standing_shear_x.at(0).size());
      int y = static_cast<int>((bounds.y + 0.5 * bounds.height) / _img.rows
          * aero::aerocv::standing_shear_x.size());
      offset = aero::aerocv::standing_shear_x.at(y).at(x);
      trans_y = aero::aerocv::standing_transform_y.at(y).at(x);
    } else {
      int x = static_cast<int>((bounds.x + 0.5 * bounds.width) / _img.cols
          * aero::aerocv::laying_shear_x.at(0).size());
      int y =  static_cast<int>((bounds.y + 0.5 * bounds.height) / _img.rows
        * aero::aerocv::laying_shear_x.size());
      offset = aero::aerocv::laying_shear_x.at(y).at(x);
      trans_y = aero::aerocv::laying_transform_y.at(y).at(x);
    }
    cv::Mat before_pad = raw.clone(); // src = dst fails, must copy
    if (offset < 0) {
      // conduct right padding
      cv::copyMakeBorder
        (before_pad, raw, 0,0,0,-offset, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
      tl = cv::Point2f(0, 0);
      tr = cv::Point2f(bounds.width, 0);
      br = cv::Point2f(bounds.width - offset, bounds.height);
      bl = cv::Point2f(-offset, bounds.height);
    } else {
      // conduct left padding
      cv::copyMakeBorder
        (before_pad, raw, 0,0,offset,0, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
      tl = cv::Point2f(offset, 0);
      tr = cv::Point2f(bounds.width + offset, 0);
      br = cv::Point2f(bounds.width, bounds.height);
      bl = cv::Point2f(0, bounds.height);
    }

    cv::Point2f src_rect[4] = {tl, tr, br, bl};
    cv::Point2f projected_rect[4] =
      {cv::Point2f(0, 0), cv::Point2f(bounds.width, 0),
       cv::Point2f(bounds.width, bounds.height + trans_y),
       cv::Point2f(0, bounds.height + trans_y)};

    // conduct projection
    auto lambda = cv::getPerspectiveTransform(src_rect, projected_rect);
    cv::Mat transformed;
    cv::warpPerspective(raw, transformed, lambda,
                        cv::Size(bounds.width, bounds.height + trans_y));
    cv::resize(transformed, *img,
               cv::Size(transformed.cols*2, transformed.rows*2), cv::INTER_CUBIC);

    if (_debug_folder != "") {
      cv::imwrite(_debug_folder + "ocr_raw"
                  + std::to_string(static_cast<int>(img - projected_images.begin()))
                  + ".jpg", raw);
      cv::imwrite(_debug_folder + "ocr_project"
                  + std::to_string(static_cast<int>(img - projected_images.begin()))
                  + ".jpg", *img);
    }

    ++img;
  }
};

//////////////////////////////////////////////////
std::vector<int> aero::aerocv::FindTargetWithOcr
(std::vector<std::string> _target_name, std::vector<aero::aerocv::objectarea> &_scene,
 cv::Mat &_img, windows::interface::WindowsInterfacePtr _windows,
 std::string _debug_folder)
{
  std::vector<cv::Mat> images(_scene.size());
  auto img = images.begin();
  for (auto obj = _scene.begin(); obj != _scene.end(); ++obj) {
    *img = _img(obj->bounds2d);
    ++img;
  }

  // with images, get OCR result
  auto ocr = _windows->OCR(images);

  // save ocr results to scene
  for (unsigned int i = 0; i < _scene.size(); ++i) {
    auto res = ocr.begin() + i;
    auto obj = _scene.begin() + i;
    for (auto txt = res->begin(); txt != res->end(); ++txt)
      if (*txt != "") // name may already have a result so push back
        obj->properties.name.push_back(*txt);
  }

  // find best matches
  std::vector<std::pair<int, int> > candidates;
  // longer match is better match, we want to find best match first
  std::sort(_target_name.begin(), _target_name.end(),
            [](std::string a, std::string b){return (a.length() > b.length());});
  for (unsigned int i = 0; i < _scene.size(); ++i) {
    auto obj = _scene.begin() + i;
    if (!obj->visible3d) continue; // for now, ignore non-visible objects
    auto res = ocr.begin() + i;
    bool go_to_next = false;
    for (auto str = _target_name.begin(); str != _target_name.end(); ++str) {
      // OCR result is first name English, second name Japanese
      std::string word = res->at(0);
      // if byte is multi byte, use second name
      if ((0x80 & (*str)[0]) != 0) word = res->at(1);
      if (word == "") { // object has no result, go to next object
        go_to_next = true;
        break;
      } else if (str->find(word) != std::string::npos) {
        candidates.push_back({i, static_cast<int>(str - _target_name.begin())});
        go_to_next = true;
        break; // found best result, go to next object 
      }
    }
    if (!go_to_next) // this means some words were at least found
      candidates.push_back({i, _target_name.size()});
  }

  if (candidates.size() == 0) // if no candidates, return
    return std::vector<int> {};

  // order candidates with best match
  std::sort(candidates.begin(), candidates.end(),
            [](std::pair<int, float> x, std::pair<int, float> y){
              return (x.second < y.second);
            });

  // divide candidates into field and add to result
  std::vector<std::vector<std::pair<int, float>> > ordered_candidates(1);
  auto oc = ordered_candidates.begin();
  float best_score = candidates.begin()->second;
  for (auto obj = candidates.begin(); obj != candidates.end(); ++obj) {
    auto s = _scene.begin() + obj->first;
    if (obj->second == best_score) {
      oc->push_back({obj->first, s->center3d.norm()});
    } else { // if not same score, add to next field
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
              [](std::pair<int, float> x, std::pair<int, float> y){
                return (x.second < y.second);
              });
    for (auto obj = c->begin(); obj != c->end(); ++obj)
      *it++ = obj->first;
  }

  return result;
}
