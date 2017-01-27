#ifndef _IK_SETTINGS_H_
#define _IK_SETTINGS_H_

#define ENUM_TO_STRING(var) #var

namespace aero
{
  enum struct arm : int {rarm, larm, either};

  enum struct ikrange : int {arm, torso, lifter, on_plane, height_only};

  enum struct eef : int {hand, grasp, pick};
}

#endif
