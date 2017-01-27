#ifndef _DEMO_Side_GRASP_INL_H_
#define _DEMO_Side_GRASP_INL_H_

#include "aero_std/ProcessFlow.h"

namespace aero
{
  struct SideGrasp
  {
    // ------------------------------
    // required parameters
    // arm, object_position, max_height
    //
    // user is allowed to configure only
    // offet_z_mid, offet_x_mid, offet_z_end, offet_x_end, maximum_grasp_width
    // ------------------------------


  public: SideGrasp() :
    arm(aero::arm::either), object_position({0.0, 0.0, 0.0}), height(0.0),
    offset_z_mid(0.0),offset_x_mid(0.0),
    offset_z_end(0.0),offset_x_end(0.0),
    default_offset_z(0.0), default_offset_x(0.05), default_offset_y(0.0),
    default_offset_x_mid(-0.1),
    default_offset_y_mid_left(0.1),
    default_offset_y_end_left(0.0) {}
    
    // arm to grasp object, "left" or "right" or "either"
  public: aero::arm arm;
    
    // object position in world coordinates
  public: Eigen::Vector3f object_position;
    
    // object parameter, object height, no use
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

  public: float default_offset_y;

   
    // default difference from end to mid
  public: float default_offset_x_mid;
  public: float default_offset_y_mid_left;

  public: float default_offset_y_end_left;
  };

  template < >
  GraspRequest Grasp<SideGrasp>(SideGrasp _grasp)
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
    Eigen::Quaternionf mid_rot = ini_rot;
    Eigen::Quaternionf end_rot = // rotate on axis Z by -M_PI/4 world
      Eigen::Quaternionf(0.92388, 0.0, 0.0, -0.382683) * mid_rot;
    if (result.arm == aero::arm::rarm) {
      //mid_rot = ini_rot;
      end_rot = // rotate on axis Y by M_PI/4 world
        Eigen::Quaternionf(0.92388, 0.0, 0.0, 0.382683) * mid_rot;
    }

    result.mid_pose.position.x = _grasp.object_position.x() + _grasp.default_offset_x + _grasp.offset_x_mid + _grasp.default_offset_x_mid;
    if (result.arm == aero::arm::larm) {
      result.mid_pose.position.y = _grasp.object_position.y() + _grasp.default_offset_y_mid_left;
    } else {
      result.mid_pose.position.y = _grasp.object_position.y() - _grasp.default_offset_y_mid_left;
    }
    result.mid_pose.position.z = _grasp.object_position.z();
    result.mid_pose.orientation.x = mid_rot.x();
    result.mid_pose.orientation.y = mid_rot.y();
    result.mid_pose.orientation.z = mid_rot.z();
    result.mid_pose.orientation.w = mid_rot.w();

    result.end_pose.position.x = _grasp.object_position.x() + _grasp.default_offset_x + _grasp.offset_x_end;
    if (result.arm == aero::arm::larm) {
    result.end_pose.position.y = _grasp.object_position.y() + _grasp.default_offset_y_end_left;
    } else { 
    result.end_pose.position.y = _grasp.object_position.y() - _grasp.default_offset_y_end_left;
    }
    result.end_pose.position.z = _grasp.object_position.z();
    result.end_pose.orientation.x = end_rot.x();
    result.end_pose.orientation.y = end_rot.y();
    result.end_pose.orientation.z = end_rot.z();
    result.end_pose.orientation.w = end_rot.w();

    return result;
    
  };

}

#endif
