/*
  @define ShoulderPitchInvTable from arm-shoulder-p offset 0
  @define ElbowPitchInvTable from arm-elbow-p offset 0
  @define WristPitchInvTable from wrist-p offset 0
  @define WristRollInvTable from wrist-r offset 0
  @define LegInvTable from lamia_s offset 0
 */

#ifndef AERO_COMMON_STROKE_TO_ANGLE_H_
#define AERO_COMMON_STROKE_TO_ANGLE_H_

#include <vector>
#include <algorithm>
#include <stdint.h>
#include "aero_hardware_interface/Stroke2Angle.hh"

namespace aero
{
  namespace common
  {

    struct S2AData
    {
      int angle;
      float stroke;
      float range;
    };

    //////////////////////////////////////////////////
    void Stroke2Angle
    (std::vector<double>& _angles, const std::vector<int16_t> _strokes)
    {
      float scale = 0.01;
 
      float right_wrist_roll_stroke =
	(scale * can_r_wrist_top + scale * can_r_wrist_bottom) * 0.5;
 
      float deg2Rad = M_PI / 180.0;
      float crotch_angle = deg2Rad * LegInvTable(scale * can_down);

      // can_order -> ros_order
      meta =
	-deg2Rad * scale * can_r_shoulder_y;
      meta =
	deg2Rad * ShoulderPitchInvTable(scale * can_r_shoulder_p);
      meta =
	deg2Rad * ElbowPitchInvTable(scale * can_r_elbow_p);
      meta =
	-deg2Rad * scale * can_r_wrist_y;
      meta =
	deg2Rad * WristPitchInvTable(
           fabs(scale * can_r_wrist_top - right_wrist_roll_stroke))
	* (can_r_wrist_top > can_r_wrist_bottom ? 1 : -1);
      meta =
	deg2Rad * WristRollInvTable(-fabs(right_wrist_roll_stroke))
	* (right_wrist_roll_stroke >= 0 ? 1 : -1);
      meta =
	deg2Rad * (scale * can_r_hand * 5.556 + 50.0);
      meta = 0;
      meta = 0;
      meta =
	-deg2Rad * (scale * can_r_hand * 5.556 + 50.0);      

      meta = 0; //upper_lower_connector
      meta = crotch_angle; //hip_joint
      meta =
	deg2Rad * LegInvTable(scale * can_up) + crotch_angle; //knee_joint
    };

  }
}

#endif
