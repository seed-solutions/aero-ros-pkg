/*
  @define ShoulderPitchInvTable from  shoulder-p offset 10.635
  @define ShoulderRollInvTable from  shoulder-r offset 0
  @define ElbowPitchInvTable from  elbow-p offset -42.426
  @define WristPitchInvTable from  wrist-p offset -9.631
  @define WristRollInvTable from  wrist-r offset -11.709
  @define WaistPitchInvTable from  waist-p offset -5.335
  @define WaistRollInvTable from  waist-r offset -5.794
  @define NeckPitchInvTable from  neck-p offset -3.124
  @define NeckRollInvTable from  neck-r offset -7.38
  @define LegInvTable from lamia offset 0
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
      float left_wrist_roll_stroke =
	(scale * can_l_wrist_top + scale * can_l_wrist_bottom) * 0.5;
      float right_wrist_roll_stroke =
	(scale * can_r_wrist_top + scale * can_r_wrist_bottom) * 0.5;
      float waist_pitch_stroke =
	(scale * can_waist_right + scale * can_waist_left) * 0.5;
      float neck_pitch_stroke =
	(scale * can_neck_right + scale * can_neck_left) * 0.5;
      float deg2Rad = M_PI / 180.0;
      float crotch_angle = deg2Rad * LegInvTable(scale * can_up);

      // can_order -> ros_order
      meta =
	deg2Rad * scale * can_waist_y;
      meta =
	deg2Rad * WaistPitchInvTable(waist_pitch_stroke);
      meta =
	deg2Rad * WaistRollInvTable(
           fabs(scale * can_waist_right - waist_pitch_stroke))
	* (can_waist_right > can_waist_left ? 1 : -1);

      meta =
	-deg2Rad * ShoulderPitchInvTable(scale * can_l_shoulder_p);
      meta =
	deg2Rad * ShoulderRollInvTable(scale * can_l_shoulder_r);
      meta =
	-deg2Rad * scale * can_l_elbow_y;
      meta =
	-deg2Rad * ElbowPitchInvTable(scale * can_l_elbow_p);
      meta =
	-deg2Rad * scale * can_l_wrist_y;
      meta =
	deg2Rad * WristPitchInvTable(
            fabs(scale * can_l_wrist_top - left_wrist_roll_stroke))
	* (can_l_wrist_top > can_l_wrist_bottom ? 1 : -1);
      meta =
	-deg2Rad * WristRollInvTable(-fabs(left_wrist_roll_stroke))
	* (left_wrist_roll_stroke >= 0 ? -1 : 1);
      meta =
	-deg2Rad * (scale * can_l_hand * 5.556 - 50.0);
      meta = 0;
      meta = 0;
      meta =
	deg2Rad * (scale * can_l_hand * 5.556 - 50.0);

      meta =
	deg2Rad * scale * can_neck_y;
      meta =
	deg2Rad * NeckPitchInvTable(neck_pitch_stroke);
      meta =
	-deg2Rad * NeckRollInvTable(
           fabs(scale * can_neck_right - neck_pitch_stroke))
	* (can_neck_right > can_neck_left ? -1 : 1);

      meta =
	-deg2Rad * ShoulderPitchInvTable(scale * can_r_shoulder_p);
      meta =
	-deg2Rad * ShoulderRollInvTable(scale * can_r_shoulder_r);
      meta =
	-deg2Rad * scale * can_r_elbow_y;
      meta =
	-deg2Rad * ElbowPitchInvTable(scale * can_r_elbow_p);
      meta =
	-deg2Rad * scale * can_r_wrist_y;
      meta =
	deg2Rad * WristPitchInvTable(
           fabs(scale * can_r_wrist_top - right_wrist_roll_stroke))
	* (can_r_wrist_top > can_r_wrist_bottom ? 1 : -1);
      meta =
	-deg2Rad * WristRollInvTable(-fabs(right_wrist_roll_stroke))
	* (right_wrist_roll_stroke >= 0 ? 1 : -1);
      meta =
	deg2Rad * (scale * can_r_hand * 5.556 - 50.0);
      meta = 0;
      meta = 0;
      meta =
	-deg2Rad * (scale * can_r_hand * 5.556 - 50.0);

      meta = 0;
      meta = crotch_angle;
      meta =
	deg2Rad * LegInvTable(scale * can_down) + crotch_angle;
    };

  }
}

#endif
