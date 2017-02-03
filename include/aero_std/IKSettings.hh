#ifndef _IK_SETTINGS_H_
#define _IK_SETTINGS_H_

#define ENUM_TO_STRING(var) #var

namespace aero
{
  enum struct arm : int {rarm, larm, either};

  enum struct ikrange : int {arm, torso, lifter, on_plane, height_only};

  enum struct eef : int {hand, grasp, pick};

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
    else if (_range == aero::ikrange::torso) mg = mg + "_with_lifter";

    return mg;
  }

  std::string armAndEEF2LinkName(aero::arm _arm, aero::eef _eef)
  {
    std::string ln = arm2LR(_arm);
    if (_eef == aero::eef::grasp) ln = ln + "_eef_grasp_link";
    else if (_eef == aero::eef::pick) ln = ln + "_eef_pick_link";

    return ln;
  }
}

#endif
