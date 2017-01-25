#ifndef _MOVEIT_GRASP_REQUEST_H_
#define _MOVEIT_GRASP_REQUEST_H_

#include "aero_std/IKSettings.hh"

namespace aero
{
  struct GraspRequest
  {

  public: GraspRequest() :
    arm(aero::arm::rarm),
    eef(aero::eef::hand),
    mid_pose(geometry_msgs::Pose()),
    mid_ik_range(aero::ikrange::arm),
    end_pose(geometry_msgs::Pose()),
    end_ik_range(aero::ikrange::arm) {}


  public: aero::arm arm;

  public: aero::eef eef;

  public: geometry_msgs::Pose mid_pose;

  public: aero::ikrange mid_ik_range; // arm, torso, lifter

  public: geometry_msgs::Pose end_pose;

  public: aero::ikrange end_ik_range;
  };
}

#endif
