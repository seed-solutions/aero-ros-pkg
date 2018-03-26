#ifndef _DEMO_SIDE_GRASP_INL_H_
#define _DEMO_SIDE_GRASP_INL_H_

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
  public: SideGrasp() : SideGrasp(aero::arm::either, aero::Vector3(0,0,0), 0.0) {}
  public: SideGrasp(aero::arm _arm, const aero::Vector3 &_object_position, double _max_height) :
    arm(_arm), object_position(_object_position), height(_max_height),
    default_offset(0.05, 0.0, 0.0),
    offset_mid(0, 0, 0),
    offset_end(0, 0, 0),
    default_offset_mid(-0.1,  0.1, 0.0),
    default_offset_end( 0.0,-0.05, 0.0)
    { }

    // arm to grasp object, "left" or "right" or "either"
  public: aero::arm arm;
    // object position in world coordinates
  public: Eigen::Vector3d object_position;
    // object parameter, object height, no use
  public: double height;
    // new
  public: aero::Vector3 offset_mid;
  public: aero::Vector3 offset_end;
  public: aero::Vector3 default_offset;
  public: aero::Vector3 default_offset_mid;
  public: aero::Vector3 default_offset_end;
  };

  template < >
  inline GraspRequest Grasp<SideGrasp>(SideGrasp &_grasp)
  {
    GraspRequest result;

    if (_grasp.arm != aero::arm::either) {
      result.arm = _grasp.arm;
    } else {
      if (_grasp.object_position.y() > 0.0) result.arm = aero::arm::larm;
      else result.arm = aero::arm::rarm;
    }

    result.eef = aero::eef::grasp;

    // compute object yaw from robot's center
    double yaw = atan2(_grasp.object_position.y(), _grasp.object_position.x());

    // object's distance
    double distance = sqrt(_grasp.object_position.y() * _grasp.object_position.y()
                           + _grasp.object_position.x() * _grasp.object_position.x());

    // compute desired yaw when object is in front of arm
    double arm_y;
    if (result.arm == aero::arm::rarm) arm_y = -0.2;
    else arm_y = 0.2;

    double des_yaw;
    if(distance == 0.0) {
      des_yaw = 0.0;
    } else {
      des_yaw = asin(arm_y / distance);
    }

    // rotate object forward
    aero::Vector3 obj_tmp = aero::Quaternion(cos((-yaw + des_yaw)/2.0), 0.0, 0.0, sin((-yaw + des_yaw)/2.0)) * _grasp.object_position;

    aero::Quaternion ini_rot_front = aero::Quaternion(1.0, 0.0, 0.0, 0.0); //reset-pose
    aero::Quaternion mid_rot_front = ini_rot_front;
    aero::Quaternion end_rot_front;

    if (result.arm == aero::arm::rarm) {
      //mid_rot = ini_rot;
      end_rot_front = // rotate on axis Y by M_PI/4 world
        aero::Quaternion(0.92388, 0.0, 0.0, 0.382683) * mid_rot_front;
    } else {
      end_rot_front = // rotate on axis Z by -M_PI/4 world
        aero::Quaternion(0.92388, 0.0, 0.0, -0.382683) * mid_rot_front;
    }

    aero::Vector3 default_offset_mid(_grasp.default_offset_mid);
    aero::Vector3 default_offset_end(_grasp.default_offset_end);
    if (result.arm == aero::arm::rarm) {
      default_offset_mid.y() = - default_offset_mid.y();
      default_offset_end.y() = - default_offset_end.y();
    }
    // compute pose in front of robot
    aero::Vector3 mid_pos_front;
    aero::Vector3 end_pos_front;
    mid_pos_front = obj_tmp + _grasp.default_offset + _grasp.offset_mid + default_offset_mid;//
    end_pos_front = obj_tmp + _grasp.default_offset + _grasp.offset_end + default_offset_end;//

    if (_grasp.object_position.z() < 0.6) { // lower object is hard to grasp
      aero::Quaternion rot_y_10(aero::Matrix3(aero::AngleAxis(M_PI/18.0, aero::Vector3::UnitY())));

      mid_rot_front = rot_y_10 * mid_rot_front;
      end_rot_front = rot_y_10 * end_rot_front;
    }

    aero::Quaternion rot_to_original_(cos((yaw - des_yaw)/2.0), 0.0, 0.0, sin((yaw - des_yaw)/2.0)); // rotate (yaw - des_yaw) :z
    aero::Transform to_original = aero::Translation::Identity() * rot_to_original_;
    result.mid_pose = to_original * (aero::Translation(mid_pos_front)*mid_rot_front);
    result.end_pose = to_original * (aero::Translation(end_pos_front)*end_rot_front);

    return result;
  };

}

#endif
