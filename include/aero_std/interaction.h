#ifndef _AERO_STD_INTERACTION_
#define _AERO_STD_INTERACTION_

#include <pcl/PCLPointCloud2.h>

namespace aero
{
  namespace aerocv
  {

    // @brief Head pose information in detected camera coordinate.
    struct PersonCameraCoords {
      Eigen::Vector3f position3d;
      // head coordinate is x axis forward, z axis up
      // pose3d converts camera coords to head coords 
      Eigen::Quaternionf pose3d;
      // information of camera pose when face was detected
      Eigen::Quaternionf mat_base_to_camera;
      Eigen::Vector3f p_base_to_camera;
    };

    // @brief From human head posture, and point location, check whether person is looking at point.
    // @param[in] _person Information of human head position and posture in camera coordinate.
    // @param[in] _point_base Target point location in base coordinate.
    // @param[in] _distance_threshold Point to sight ray distance in meters to define as looking.
    // @return Looking or not looking.
    bool SharedAttention
      (PersonCameraCoords _person, Eigen::Vector3f _point_base, float _distance_threshold)
    {
      Eigen::Vector3f sight_direction_camera = _person.pose3d * Eigen::Vector3f(1.0, 0, 0);
      Eigen::Vector3f sight_direction_base = _person.mat_base_to_camera * sight_direction_camera;
      float distance = (sight_direction_base.cross(_person.mat_base_to_camera * _person.position3d
                                                   + _person.p_base_to_camera - _point_base)).norm()
        / sight_direction_camera.norm();

      if (distance < _distance_threshold) return true;
      else return false;
    };

    // @brief From human head posture, finds the point cloud region that the human is looking at.
    // @param[in] _person Information of human head position and posture in camera coordinate.
    // @param[in] _points_camera Point cloud to be edited. Points not in shared attention will be removed.
    // @param[in] _mat_base_to_camera Camera orientation information where point cloud was captured.
    // @param[in] _p_base_to_camera Camera position information where point cloud was captured.
    // @param[in] _threshold Point to sight ray distance in meters to include as looking.
    // @param[in] _transform_points If true, returns points in base coordinate. Else, camera coordinate. 
    void SharedAttention
      (PersonCameraCoords _person, pcl::PointCloud<pcl::PointXYZRGB>::Ptr _points_camera,
       Eigen::Quaternionf _mat_base_to_camera, Eigen::Vector3f _p_base_to_camera, float _threshold,
       bool _transform_points=false)
    {
      Eigen::Vector3f sight_direction_camera = _person.pose3d * Eigen::Vector3f(1.0, 0, 0);
      Eigen::Vector3f sight_direction_base = _person.mat_base_to_camera * sight_direction_camera;

      for (auto it = _points_camera->points.begin(); it != _points_camera->points.end(); ) {
        if (std::isnan(it->x) || std::isnan(it->y) || std::isnan(it->z)) {
          it = _points_camera->points.erase(it);
          continue;
        }

        Eigen::Vector3f point_base =
          _mat_base_to_camera * (Eigen::Vector3f(it->x, it->y, it->z)) + _p_base_to_camera;
        float distance =  (sight_direction_base.cross(_person.mat_base_to_camera * _person.position3d
                                                      + _person.p_base_to_camera - point_base)).norm()
          / sight_direction_camera.norm();
        if (distance > _threshold) {
          it = _points_camera->points.erase(it);
          continue;
        }

        if (_transform_points) {
          it->x = point_base.x();
          it->y = point_base.y();
          it->z = point_base.z();
        }

        ++it;
      }

      _points_camera->width = _points_camera->points.size();
      _points_camera->height = 1;
    };

    // @brief From human head posture, and plane location, check whether person is looking at plane.
    // @param[in] _person Information of human head position and posture in camera coordinate.
    // @param[in] _plane_normal Normal of plane in base coordinate or global coordinate.
    // @param[in] _plane_center Position of plane in base coordinate or global coordinate
    // @param[in] _inner_threshold Distance range in meters to define as absolutely looking at plane.
    // @param[in] _outer_threshold Distance range in meters to define as likely looking at plane.
    // @param[in] _mat_global_to_base When applied, _plane_normal will be parsed as global coordinate.
    // @param[in] _p_global_to_base When applied, _plane_normal will be parsed as global coordinate.
    // @return Score of looking or not.
    float SharedAttention
      (PersonCameraCoords _person, Eigen::Vector3f _plane_normal, Eigen::Vector3f _plane_center,
       float _inner_threshold, float _outer_threshold,
       Eigen::Quaternionf _mat_global_to_base=Eigen::Quaternionf(0, 0, 0, 0),
       Eigen::Vector3f _p_global_to_base=Eigen::Vector3f(0, 0, 0))
    {
      Eigen::Vector3f plane_normal;
      Eigen::Vector3f plane_center;

      // check global or base coordinate
      if (_mat_global_to_base.norm() < 0.00001 && _p_global_to_base.norm() < 0.00001) {
        plane_normal = _plane_normal;
        plane_center = _plane_center;
      } else {
        plane_normal = _mat_global_to_base.inverse() * _plane_normal;
        plane_center = _mat_global_to_base.inverse() * (_plane_center - _p_global_to_base);
      }

      Eigen::Vector3f sight_direction_camera = _person.pose3d * Eigen::Vector3f(1.0, 0, 0);
      Eigen::Vector3f sight_direction_base = _person.mat_base_to_camera * sight_direction_camera;
      Eigen::Vector3f person_position3d_base =
        _person.mat_base_to_camera * _person.position3d + _person.p_base_to_camera;

      float t = plane_normal.dot(person_position3d_base - plane_center)
        / plane_normal.dot(sight_direction_base);

      Eigen::Vector3f intersection = person_position3d_base + t * sight_direction_base;
      float distance = (intersection - plane_center).norm();

      if (distance < _inner_threshold)
        return 1.0;
      else if (distance < _outer_threshold)
        return 1.0 - (distance - _inner_threshold) / (_outer_threshold - _inner_threshold);

      return 0.0;
    };

  }
}

#endif
