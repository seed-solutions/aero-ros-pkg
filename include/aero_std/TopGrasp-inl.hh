#ifndef _DEMO_TOP_GRASP_INL_H_
#define _DEMO_TOP_GRASP_INL_H_

#include "aero_std/ProcessFlow.h"

namespace aero
{
  struct TopGrasp
  {
    // ------------------------------
    // required parameters
    // arm, object_position, max_height
    //
    // user is allowed to configure only
    // offet_z_mid, offet_x_mid, offet_z_end, offet_x_end, maximum_grasp_width
    // ------------------------------


  public: TopGrasp() :
    arm(aero::arm::either), object_position({0.0, 0.0, 0.0}), height(0.0),
    offset_z_mid(0.0),offset_x_mid(0.0),
    offset_z_end(0.0),offset_x_end(0.0),
    default_offset_z(0.0), default_offset_x(0.0),
    default_offset_z_mid(0.1),
    maximum_grasp_width(0.15) {}
    
    // arm to grasp object, "left" or "right" or "either"
  public: aero::arm arm;
    
    // object position used to solve grasp pose , world coordinates
  public: Eigen::Vector3f object_position;
    
    // object parameter, object height
  public: float height;

    
    // user parameter, set high to avoid object finger collision
  public: float offset_z_mid;

  public: float offset_x_mid;

    // user parameter
  public: float offset_z_end;

  public: float offset_x_end;

    // default offset from ik eef position
  public: float default_offset_z;

  public: float default_offset_x;


    // default height difference from end to mid
  public: float default_offset_z_mid;

    // physical parameter from the degree of opening of hand
  public: float maximum_grasp_width;

  };

  template < >
  GraspRequest Grasp<TopGrasp>(TopGrasp _grasp)
  {
    GraspRequest result;
    if (_grasp.arm != aero::arm::either) {
      result.arm = _grasp.arm;
    } else {
      if (_grasp.object_position.y() > 0.0) result.arm = aero::arm::larm;
      else result.arm = aero::arm::rarm;
    }

    result.eef = aero::eef::grasp;

    Eigen::Quaternionf ini_rot = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0); //reset-pose
    Eigen::Quaternionf mid_rot =
      Eigen::Quaternionf(0.707107, 0.707107, 0.0, 0.0) * ini_rot;
    Eigen::Quaternionf end_rot = // rotate on axis Y by -M_PI/4 world
      Eigen::Quaternionf(0.92388, 0.0, 0.382683, 0.0) * mid_rot;
    if (result.arm == aero::arm::rarm) {
      mid_rot =
        Eigen::Quaternionf(0.707107, -0.707107, 0.0, 0.0) * ini_rot;
      end_rot = // rotate on axis Y by M_PI/4 world
        Eigen::Quaternionf(0.92388, 0.0, 0.382683, 0.0) * mid_rot;
    }

    result.mid_pose.position.x = _grasp.object_position.x() + _grasp.default_offset_x + _grasp.offset_x_mid;
    result.mid_pose.position.y = _grasp.object_position.y();
    result.mid_pose.position.z = _grasp.object_position.z() + _grasp.height/2.0 + _grasp.default_offset_z + _grasp.default_offset_z_mid;
    result.mid_pose.orientation.x = mid_rot.x();
    result.mid_pose.orientation.y = mid_rot.y();
    result.mid_pose.orientation.z = mid_rot.z();
    result.mid_pose.orientation.w = mid_rot.w();

    result.end_pose.position.x = _grasp.object_position.x() + _grasp.default_offset_x + _grasp.offset_x_end+ _grasp.maximum_grasp_width;
    result.end_pose.position.y = _grasp.object_position.y();
    result.end_pose.position.z = _grasp.object_position.z() + _grasp.default_offset_z;
    result.end_pose.orientation.x = end_rot.x();
    result.end_pose.orientation.y = end_rot.y();
    result.end_pose.orientation.z = end_rot.z();
    result.end_pose.orientation.w = end_rot.w();

    return result;
    
  };

}

#endif
