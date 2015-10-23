/*
 * This file is modified with a script. Do not Edit!
 */

#ifndef _STROKE_TO_ANGLE_H_
#define _STROKE_TO_ANGLE_H_

#include <ros/ros.h>

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
    (std::vector<double>& _angles, const std::vector<double> _strokes)
    {
      float leftWristRollStroke =
	(l_wrist_top_joint + l_wrist_bottom_joint) * 0.5;
      float rightWristRollStroke =
	(r_wrist_top_joint + r_wrist_bottom_joint) * 0.5;
      float waistPitchStroke =
	(waist_right_joint + waist_left_joint) * 0.5;
      float neckPitchStroke =
	(neck_right_joint + neck_left_joint) * 0.5;
      float deg2Rad = M_PI / 180.0;

      // can_order -> ros_order
      meta =
	deg2Rad * waist_pitch_joint;
      meta =
	deg2Rad * WaistPitchInvTable(waistPitchStroke);
      meta =
	deg2Rad * WaistRollInvTable(fabs(waist_right_joint - waistPitchStroke))
	* (waist_right_joint > waist_left_joint ? 1 : -1);

      meta =
	-deg2Rad * ShoulderPitchInvTable(l_shoulder_pitch_joint);
      meta =
	deg2Rad * ShoulderRollInvTable(l_shoulder_roll_joint);
      meta =
	-deg2Rad * l_elbow_yaw_joint;
      meta =
	-deg2Rad * ElbowPitchInvTable(l_elbow_pitch_joint);
      meta =
	-deg2Rad * l_wrist_roll_joint;
      meta =
	deg2Rad * WristPitchInvTable(fabs(l_wrist_top_joint - leftWristRollStroke))
	* (l_wrist_top_joint > l_wrist_bottom_joint ? 1 : -1);
      meta =
	deg2Rad * WristRollInvTable(-fabs(leftWristRollStroke))
	* (leftWristRollStroke >= 0 ? -1 : 1);
      meta =
	-deg2Rad * (l_hand_joint * 5.556 + 50.0);
      meta = 0;
      meta = 0;
      meta =
	deg2Rad * (l_hand_joint * 5.556 + 50.0);

      meta =
	deg2Rad * neck_yaw_joint;
      meta =
	deg2Rad * NeckPitchInvTable(neckPitchStroke);
      meta =
	deg2Rad * NeckRollInvTable(fabs(neck_right_joint - neckPitchStroke))
	* (neck_right_joint > neck_left_joint ? -1 : 1);

      meta =
	-deg2Rad * ShoulderPitchInvTable(r_shoulder_pitch_joint);
      meta =
	-deg2Rad * ShoulderRollInvTable(r_shoulder_roll_joint);
      meta =
	-deg2Rad * r_elbow_yaw_joint;
      meta =
	-deg2Rad * ElbowPitchInvTable(r_elbow_pitch_joint);
      meta =
	-deg2Rad * r_wrist_roll_joint;
      meta =
	deg2Rad * WristPitchInvTable(fabs(r_wrist_top_joint - rightWristRollStroke))
	* (r_wrist_top_joint > r_wrist_bottom_joint ? 1 : -1);
      meta =
	deg2Rad * WristRollInvTable(-fabs(rightWristRollStroke))
	* (rightWristRollStroke >= 0 ? 1 : -1);
      meta =
	deg2Rad * (r_hand_joint * 8.475 + 50.0);
      meta = 0;
      meta = 0;
      meta =
	-deg2Rad * (r_hand_joint * 8.475 + 50.0);

      meta = 0;
      meta =
	-deg2Rad * f_l_crotch_yaw_joint;
      meta =
	deg2Rad * CrotchPitchInvTable(f_l_crotch_pitch_joint);
      meta =
	deg2Rad * KneePitchInvTable(f_l_knee_pitch_joint);
      meta =
	-deg2Rad * r_l_crotch_yaw_joint;
      meta =
	deg2Rad * CrotchPitchInvTable(r_l_crotch_pitch_joint);
      meta =
	deg2Rad * KneePitchInvTable(r_l_knee_pitch_joint);
      meta =
	-deg2Rad * f_r_crotch_yaw_joint;
      meta =
	deg2Rad * CrotchPitchInvTable(f_r_crotch_pitch_joint);
      meta =
	deg2Rad * KneePitchInvTable(f_r_knee_pitch_joint);
      meta =
	-deg2Rad * r_r_crotch_yaw_joint;
      meta =
	deg2Rad * CrotchPitchInvTable(r_r_crotch_pitch_joint);
      meta =
	deg2Rad * KneePitchInvTable(r_r_knee_pitch_joint);
    };

    //////////////////////////////////////////////////
    float TableTemplate (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      std::vector<S2AData> candidates;
      std::vector<S2AData> appendix;

      switch (roundedStroke)
      {
      default: return 0.0;
      }

      if (_stroke < 0)
      {
	if (candidates.size() >= 2)
	  if (candidates[0].stroke < candidates[1].stroke)
	    std::reverse(candidates.begin(), candidates.end());

	for (unsigned int i = 0; i < candidates.size(); ++i)
	  if (_stroke >= candidates[i].stroke)
	    return candidates[i].angle
	      - (candidates[i].stroke - _stroke) / candidates[i].range;

	if (appendix.size() >= 2)
	  if (appendix[0].stroke < appendix[1].stroke)
	    std::reverse(appendix.begin(), appendix.end());

	if (appendix.size() == 0)
	  return candidates[candidates.size() - 1].angle;
	else
	  return appendix[0].angle
	    - (appendix[0].stroke - _stroke) / appendix[0].range;
      }
      else
      {
	if (candidates.size() >= 2)
	  if (candidates[0].stroke > candidates[1].stroke)
	    std::reverse(candidates.begin(), candidates.end());

	for (unsigned int i = 0; i < candidates.size(); ++i)
	  if (_stroke <= candidates[i].stroke)
	    if (candidates[i].range == 0)
	      return candidates[i].angle;
	    else
	      return candidates[i].angle
		- (candidates[i].stroke - _stroke) / candidates[i].range;

	if (appendix.size() >= 2)
	  if (appendix[0].stroke > appendix[1].stroke)
	    std::reverse(appendix.begin(), appendix.end());

	if (appendix.size() == 0)
	{
	  return candidates[candidates.size() - 1].angle;
	}
	else
	{
	  if (appendix[0].range == 0)
	    return appendix[0].angle;
	  else
	    return appendix[0].angle
	      - (appendix[0].stroke - _stroke) / appendix[0].range;
	}
      }
    };

  }
}

#endif
