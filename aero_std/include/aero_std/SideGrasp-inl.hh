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


  public: SideGrasp() :
    arm(aero::arm::either), object_position({0.0, 0.0, 0.0}), height(0.0),
    offset_z_mid(0.0),offset_x_mid(0.0),
    offset_z_end(0.0),offset_x_end(0.0),
    default_offset_z(0.0), default_offset_x(0.05), default_offset_y(0.0),
    default_offset_x_mid(-0.1),
    default_offset_y_mid_left(0.1),
    default_offset_y_end_left(-0.05) {}
    
    // arm to grasp object, "left" or "right" or "either"
  public: aero::arm arm;
    
    // object position in world coordinates
  public: Eigen::Vector3d object_position;
    
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
  inline GraspRequest Grasp<SideGrasp>(SideGrasp _grasp)
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
    double des_yaw = asin(arm_y / distance);

    // rotate object forward
    Eigen::Vector3d obj_tmp = Eigen::Quaterniond(cos((-yaw + des_yaw)/2.0), 0.0, 0.0, sin((-yaw + des_yaw)/2.0)) * _grasp.object_position;



    Eigen::Quaterniond ini_rot_front = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0); //reset-pose
    Eigen::Quaterniond mid_rot_front = ini_rot_front;
    Eigen::Quaterniond end_rot_front = // rotate on axis Z by -M_PI/4 world
      Eigen::Quaterniond(0.92388, 0.0, 0.0, -0.382683) * mid_rot_front;
    if (result.arm == aero::arm::rarm) {
      //mid_rot = ini_rot;
      end_rot_front = // rotate on axis Y by M_PI/4 world
        Eigen::Quaterniond(0.92388, 0.0, 0.0, 0.382683) * mid_rot_front;
    }


    // compute pose in front of robot
    Eigen::Vector3d mid_pos_front;
    Eigen::Vector3d end_pos_front;

    mid_pos_front.x() = obj_tmp.x() + _grasp.default_offset_x + _grasp.offset_x_mid + _grasp.default_offset_x_mid;
    if (result.arm == aero::arm::larm) mid_pos_front.y() = obj_tmp.y() + _grasp.default_offset_y_mid_left;
    else mid_pos_front.y() = obj_tmp.y() - _grasp.default_offset_y_mid_left;
    mid_pos_front.z() = obj_tmp.z();

    end_pos_front.x() = obj_tmp.x() + _grasp.default_offset_x + _grasp.offset_x_end;
    if (result.arm == aero::arm::larm) end_pos_front.y() = obj_tmp.y() + _grasp.default_offset_y_end_left;
    else end_pos_front.y() = obj_tmp.y() - _grasp.default_offset_y_end_left;
    end_pos_front.z() = obj_tmp.z();

    if (_grasp.object_position.z() < 0.6) { // lower object is hard to grasp
      Eigen::Quaterniond rot_y_10(Eigen::Matrix3d(Eigen::AngleAxisd(M_PI/18.0, Eigen::Vector3d::UnitY())));
      mid_rot_front = rot_y_10 * mid_rot_front;
      end_rot_front = rot_y_10 * end_rot_front;
      mid_pos_front.z() = mid_pos_front.z();
      end_pos_front.z() = end_pos_front.z();
    }

    Eigen::Quaterniond rot_to_original(cos((yaw - des_yaw)/2.0), 0.0, 0.0, sin((yaw - des_yaw)/2.0));
    Eigen::Vector3d mid_pos = rot_to_original * mid_pos_front;
    Eigen::Vector3d end_pos = rot_to_original * end_pos_front;
    Eigen::Quaterniond mid_rot = rot_to_original * mid_rot_front;
    Eigen::Quaterniond end_rot = rot_to_original * end_rot_front;

    result.mid_pose.position.x = mid_pos.x();
    result.mid_pose.position.y = mid_pos.y();
    result.mid_pose.position.z = mid_pos.z();
    result.mid_pose.orientation.w = mid_rot.w();
    result.mid_pose.orientation.x = mid_rot.x();
    result.mid_pose.orientation.y = mid_rot.y();
    result.mid_pose.orientation.z = mid_rot.z();

    result.end_pose.position.x = end_pos.x();
    result.end_pose.position.y = end_pos.y();
    result.end_pose.position.z = end_pos.z();
    result.end_pose.orientation.w = end_rot.w();
    result.end_pose.orientation.x = end_rot.x();
    result.end_pose.orientation.y = end_rot.y();
    result.end_pose.orientation.z = end_rot.z();
    return result;
    
  };

}

#endif
