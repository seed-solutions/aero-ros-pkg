/*
  @define ShoulderPitchTable from arm-shoulder-p offset 0
  @define ElbowPitchTable from arm-elbow-p offset 0
  @define WristRollPitchTable from wrist-p wrist-r symmetric 1
  @define LegTable from lamia_s offset 0
 */

#ifndef AERO_COMMON_ANGLE_TO_STROKE_H_
#define AERO_COMMON_ANGLE_TO_STROKE_H_

#include <vector>
#include <algorithm>
#include <stdint.h>
#include "aero_hardware_interface/Angle2Stroke.hh"

namespace aero
{
  namespace common
  {

    struct dualJoint
    {
      float one;
      float two;
    };

    //////////////////////////////////////////////////
    void Angle2Stroke
    (std::vector<int16_t>& _strokes, const std::vector<double> _angles)
    {
      float rad2Deg = 180.0 / M_PI;
      float scale = 100.0;
      dualJoint right_wrist =
	WristRollPitchTable(rad2Deg * r_wrist_r_joint,
			    rad2Deg * -r_wrist_p_joint);

      // ros_order -> can_order
      meta = -scale * rad2Deg * r_shoulder_y_joint; //0
      meta =
	scale * ShoulderPitchTable(rad2Deg * r_shoulder_p_joint); //1
      meta = scale * ElbowPitchTable(rad2Deg * r_elbow_joint); //2
      meta = -scale * rad2Deg * r_wrist_y_joint; //3
      meta = scale * right_wrist.one; //4
      meta = scale * right_wrist.two; //5
      meta = -scale * (rad2Deg * r_thumb_joint + 50.0) * 0.18;//6

      meta = scale * LegTable(rad2Deg * (knee_joint - hip_joint)); //0
      meta = scale * LegTable(rad2Deg * hip_joint);  //1
    };

  }
}

#endif
