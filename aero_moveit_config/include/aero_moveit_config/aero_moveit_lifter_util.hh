#ifndef _AERO_MOVEIT_LIFTER_UTIL_
#define _AERO_MOVEIT_LIFTER_UTIL_
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace aero {
  namespace lifter {
    void copyBounds(const moveit::core::VariableBounds &from, moveit::core::VariableBounds &to) {
      to.acceleration_bounded_ = from.acceleration_bounded_;
      to.max_acceleration_ = from.max_acceleration_;
      to.max_velocity_ = from.max_velocity_;
      to.max_position_ = from.max_position_;
      to.min_acceleration_ = from.min_acceleration_;
      to.min_velocity_ = from.min_velocity_;
      to.min_position_ = from.min_position_;
      to.position_bounded_ = from.position_bounded_;
      to.velocity_bounded_ = from.velocity_bounded_;
    }
    void switchHeightOnly(robot_state::RobotModelPtr model) {
      const std::vector<robot_model::JointModel*> jm = model->getJointModels();
      std::vector<moveit::core::VariableBounds> bound_original;
      moveit::core::VariableBounds bound;
      for (int i = 0; i < jm.size(); ++i) {
        if (jm[i]->getName() == "virtual_lifter_x_joint") {
          bound_original = jm[i]->getVariableBounds();
          aero::lifter::copyBounds(bound_original[0], bound);
          bound.max_position_ = 0.0;
          bound.min_position_ = 0.0;
          jm[i]->setVariableBounds("virtual_lifter_x_joint", bound);
        } else if (jm[i]->getName() == "virtual_lifter_z_joint") {
          bound_original = jm[i]->getVariableBounds();
          aero::lifter::copyBounds(bound_original[0], bound);
          bound.max_position_ = 0.0;
          bound.min_position_ = -0.4;
          jm[i]->setVariableBounds("virtual_lifter_z_joint", bound);
        }
      }
    }

    void switchOnPlane(robot_state::RobotModelPtr model) {
      const std::vector<robot_model::JointModel*> jm = model->getJointModels();
      std::vector<moveit::core::VariableBounds> bound_original;
      moveit::core::VariableBounds bound;
      for (int i = 0; i < jm.size(); ++i) {
        if (jm[i]->getName() == "virtual_lifter_x_joint") {
          bound_original = jm[i]->getVariableBounds();
          aero::lifter::copyBounds(bound_original[0], bound);
          bound.max_position_ = 0.2;
          bound.min_position_ = -0.2;
          jm[i]->setVariableBounds("virtual_lifter_x_joint", bound);
        } else if (jm[i]->getName() == "virtual_lifter_z_joint") {
          bound_original = jm[i]->getVariableBounds();
          aero::lifter::copyBounds(bound_original[0], bound);
          bound.max_position_ = -0.08;
          bound.min_position_ = -0.3;
          jm[i]->setVariableBounds("virtual_lifter_z_joint", bound);
        }
      }
    }
    
  }
}

#endif
