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
}

#endif
