/*
 * This file is modified with a script. Do not Edit!
 */

#ifndef _STROKE_TO_ANGLE_H_
#define _STROKE_TO_ANGLE_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>

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
    (sensor_msgs::JointState& _angles,
     const pr2_controllers_msgs::JointTrajectoryControllerState _strokes)
    {
      float leftWristRollStroke =
	(l_wrist_top_joint + l_wrist_bottom_joint) * 0.5;
      float rightWristRollStroke =
	(r_wrist_top_joint + r_wrist_bottom_joint) * 0.5;
      float waistPitchStroke =
	(waist_right_joint + waist_left_joint) * 0.5;
      float neckPitchStroke =
	(neck_right_joint + neck_left_joint) * 0.5;
      float deg2Theta = M_PI / 180.0;

      meta =
	deg2Theta * waist_pitch_joint;
      meta =
	deg2Theta * WaistPitchInvTable(waistPitchStroke);
      meta =
	deg2Theta * WaistRollInvTable(fabs(waist_right_joint - waistPitchStroke))
	* (waist_right_joint > waist_left_joint ? 1 : -1);

      meta =
	-deg2Theta * ShoulderPitchInvTable(l_shoulder_pitch_joint);
      meta =
	deg2Theta * ShoulderRollInvTable(l_shoulder_roll_joint);
      meta =
	-deg2Theta * l_elbow_yaw_joint;
      meta =
	-deg2Theta * ElbowPitchInvTable(l_elbow_pitch_joint);
      meta =
	-deg2Theta * l_wrist_roll_joint;
      meta =
	deg2Theta * WristPitchInvTable(fabs(l_wrist_top_joint - leftWristRollStroke))
	* (l_wrist_top_joint > l_wrist_bottom_joint ? 1 : -1);
      meta =
	deg2Theta * WristRollInvTable(-fabs(leftWristRollStroke))
	* (leftWristRollStroke >= 0 ? -1 : 1);
      meta =
	-deg2Theta * (l_hand_joint * 5.556 + 50.0);
      meta = 0;
      meta = 0;
      meta = 0;

      meta =
	deg2Theta * neck_yaw_joint;
      meta =
	deg2Theta * NeckPitchInvTable(neckPitchStroke);
      meta =
	deg2Theta * NeckRollInvTable(fabs(neck_right_joint - neckPitchStroke))
	* (neck_right_joint > neck_left_joint ? -1 : 1);

      meta =
	-deg2Theta * ShoulderPitchInvTable(r_shoulder_pitch_joint);
      meta =
	-deg2Theta * ShoulderRollInvTable(r_shoulder_roll_joint);
      meta =
	-deg2Theta * r_elbow_yaw_joint;
      meta =
	-deg2Theta * ElbowPitchInvTable(r_elbow_pitch_joint);
      meta =
	-deg2Theta * r_wrist_roll_joint;
      meta =
	deg2Theta * WristPitchInvTable(fabs(r_wrist_top_joint - rightWristRollStroke))
	* (r_wrist_top_joint > r_wrist_bottom_joint ? 1 : -1);
      meta =
	deg2Theta * WristRollInvTable(-fabs(rightWristRollStroke))
	* (rightWristRollStroke >= 0 ? 1 : -1);
      meta =
	deg2Theta * (r_hand_joint * 5.556 + 50.0);
      meta = 0;
      meta = 0;
      meta = 0;

      meta = 0;
      meta =
	-deg2Theta * f_l_crotch_yaw_joint;
      meta =
	deg2Theta * CrotchPitchInvTable(f_l_crotch_pitch_joint);
      meta =
	deg2Theta * KneePitchInvTable(f_l_knee_pitch_joint);
      meta =
	-deg2Theta * r_l_crotch_yaw_joint;
      meta =
	deg2Theta * CrotchPitchInvTable(r_l_crotch_pitch_joint);
      meta =
	deg2Theta * KneePitchInvTable(r_l_knee_pitch_joint);
      meta =
	-deg2Theta * f_r_crotch_yaw_joint;
      meta =
	deg2Theta * CrotchPitchInvTable(f_r_crotch_pitch_joint);
      meta =
	deg2Theta * KneePitchInvTable(f_r_knee_pitch_joint);
      meta =
	-deg2Theta * r_r_crotch_yaw_joint;
      meta =
	deg2Theta * CrotchPitchInvTable(r_r_crotch_pitch_joint);
      meta =
	deg2Theta * KneePitchInvTable(r_r_knee_pitch_joint);
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
