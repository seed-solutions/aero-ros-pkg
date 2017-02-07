#ifndef _IK_SETTINGS_H_
#define _IK_SETTINGS_H_

#define ENUM_TO_STRING(var) #var

#include <unordered_map>

namespace aero
{
  enum struct arm : int {rarm, larm, either};

  enum struct ikrange : int {arm, torso, lifter, on_plane, height_only};

  enum struct eef : int {hand, grasp, pick};

  enum struct joint : int {r_shoulder_p,
      r_shoulder_r,
      r_shoulder_y,
      r_elbow,
      r_wrist_y,
      r_wrist_p,
      r_wrist_r,
      l_shouldel_p,
      l_shouldel_r,
      l_shouldel_y,
      l_elbow,
      l_wrist_y,
      l_wrist_p,
      l_wrist_r,
      waist_y,
      waist_p,
      waist_r,
      neck_y,
      neck_p,
      neck_r,
      lifter_x,
      lifter_z
      };

  std::map<aero::joint, std::string> joint_map = {
    {aero::joint::r_shoulder_p,"r_shoulder_p_joint"},
    {aero::joint::r_shoulder_r,"r_shoulder_r_joint"},
    {aero::joint::r_shoulder_y,"r_shoulder_y_joint"},
    {aero::joint::r_elbow,"r_elbow_joint"},
    {aero::joint::r_wrist_y,"r_wrist_y_joint"},
    {aero::joint::r_wrist_p,"r_wrist_p_joint"},
    {aero::joint::r_wrist_r,"r_wrist_r_joint"},
    {aero::joint::l_shouldel_p,"l_shouldel_p_joint"},
    {aero::joint::l_shouldel_r,"l_shouldel_l_joint"},
    {aero::joint::l_shouldel_y,"l_shouldel_y_joint"},
    {aero::joint::l_elbow,"l_elbow_joint"},
    {aero::joint::l_wrist_y,"l_wrist_y_joint"},
    {aero::joint::l_wrist_p,"l_wrist_p_joint"},
    {aero::joint::l_wrist_r,"l_wrist_l_joint"},
    {aero::joint::waist_y,"waist_y_joint"},
    {aero::joint::waist_p,"waist_p_joint"},
    {aero::joint::waist_r,"waist_r_joint"},
    {aero::joint::neck_y,"neck_y_joint"},
    {aero::joint::neck_y,"neck_y_joint"},
    {aero::joint::lifter_x,"virtual_lifter_x_joint"},
    {aero::joint::lifter_z,"virtual_lifter_z_joint"}
  };

  std::string arm2LR(aero::arm _arm)
  {
    if (_arm == aero::arm::rarm) return "r";
    else if (_arm == aero::arm::larm) return "l";
    else return "";
  }

  std::string arm2LeftRight(aero::arm _arm)
  {
    if (_arm == aero::arm::rarm) return "right";
    else if (_arm == aero::arm::larm) return "left";
    else return "";
  }

  std::string arm2LarmRarm(aero::arm _arm)
  {
    if (_arm == aero::arm::rarm) return "rarm";
    else if (_arm == aero::arm::larm) return "larm";
    else return "";
  }

  std::string armAndRange2MoveGroup(aero::arm _arm, aero::ikrange _range)
  {
    std::string mg = arm2LarmRarm(_arm);
    if (_range == aero::ikrange::torso) mg = mg + "_with_torso";
    else if (_range == aero::ikrange::lifter) mg = mg + "_with_lifter";

    return mg;
  }

  std::string armAndEEF2LinkName(aero::arm _arm, aero::eef _eef)
  {
    std::string ln = arm2LR(_arm);
    if (_eef == aero::eef::grasp) ln = ln + "_eef_grasp_link";
    else if (_eef == aero::eef::pick) ln = ln + "_eef_pick_link";

    return ln;
  }

  std::string joint2JointName(aero::joint _joint)
  {
    return aero::joint_map[_joint];
  }
}

#endif
