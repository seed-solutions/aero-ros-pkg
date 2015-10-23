/*
 * This file is modified with a script. Do not Edit!
 */

#ifndef _ANGLE_TO_STROKE_H_
#define _ANGLE_TO_STROKE_H_

#include <ros/ros.h>

namespace aero
{
  namespace common
  {
    //////////////////////////////////////////////////
    struct dualJoint
    {
      float one;
      float two;
    };

    //////////////////////////////////////////////////
    void Angle2Stroke
    (std::vector<double>& _strokes, const std::vector<double> _angles)
    {
      float rad2Deg = 180.0 / M_PI;
      dualJoint rightWrist =
	WristRollPitchTable(rad2Deg * r_wrist_p_joint,
			    rad2Deg * -r_wrist_r_joint);
      dualJoint leftWrist =
	WristRollPitchTable(rad2Deg * l_wrist_p_joint,
			    rad2Deg * l_wrist_r_joint);
      dualJoint waist =
	WaistRollPitchTable(-rad2Deg * waist_r_joint,
			    rad2Deg * waist_p_joint);
      dualJoint neck =
	NeckRollPitchTable(-rad2Deg * neck_r_joint,
			   rad2Deg * neck_p_joint);

      // ros_order -> can_order
      meta = rad2Deg * neck_y_joint;
      meta = neck.one;
      meta = neck.two;

      meta =
	ShoulderPitchTable(-rad2Deg * r_shoulder_p_joint);
      meta =
	ShoulderRollTable(-rad2Deg * r_shoulder_r_joint);
      meta = -rad2Deg * r_shoulder_y_joint;
      meta = ElbowPitchTable(-rad2Deg * r_elbow_joint);
      meta = -rad2Deg * r_wrist_y_joint;
      meta = rightWrist.one;
      meta = rightWrist.two;
      meta = - (rad2Deg * r_thumb_joint + 50.0) * 0.18;

      meta =
	ShoulderPitchTable(-rad2Deg * l_shoulder_p_joint);
      meta =
	ShoulderRollTable(rad2Deg * l_shoulder_r_joint);
      meta = -rad2Deg * l_shoulder_y_joint;
      meta = ElbowPitchTable(-rad2Deg * l_elbow_joint);
      meta = -rad2Deg * l_wrist_y_joint;
      meta = leftWrist.one;
      meta = leftWrist.two;
      meta = (rad2Deg * l_thumb_joint - 50.0) * 0.18;

      meta = rad2Deg * waist_y_joint;
      meta = waist.one;
      meta = waist.two;

      meta = -rad2Deg * fr_hip_y_joint;
      meta = CrotchPitchTable(rad2Deg * fr_hip_p_joint);
      meta = KneePitchTable(rad2Deg * fr_knee_joint);
      meta = -rad2Deg * rr_hip_y_joint;
      meta = CrotchPitchTable(rad2Deg * rr_hip_p_joint);
      meta = KneePitchTable(rad2Deg * rr_knee_joint);
      meta = -rad2Deg * fl_hip_y_joint;
      meta = CrotchPitchTable(rad2Deg * fl_hip_p_joint);
      meta = KneePitchTable(rad2Deg * fl_knee_joint);
      meta = -rad2Deg * rl_hip_y_joint;
      meta = CrotchPitchTable(rad2Deg * rl_hip_p_joint);
      meta = KneePitchTable(rad2Deg * rl_knee_joint);
    };

    //////////////////////////////////////////////////
    float TableTemplate (float _angle)
    {
      int roundedAngle = static_cast<int>(_angle);
      if (_angle > roundedAngle + 0.001) ++roundedAngle;
      float stroke;
      float interval;

      switch (roundedAngle)
      {
      default: return 0.0;
      }

      return stroke - (roundedAngle - _angle) * interval;
    };

    //////////////////////////////////////////////////
    dualJoint TableTemplate (float _angle1, float _angle2)
    {
      int roundedAngle1 = static_cast<int>(_angle1);
      if (_angle1 < 0)
      {
	if (_angle1 < roundedAngle1 - 0.001) --roundedAngle1;
      }
      else
      {
	if (_angle1 > roundedAngle1 + 0.001) ++roundedAngle1;
      }

      int roundedAngle2 = static_cast<int>(_angle2);
      if (_angle2 < 0)
      {
	if (_angle2 < roundedAngle2 - 0.001) --roundedAngle2;
      }
      else
      {
	if (_angle2 > roundedAngle2 + 0.001) ++roundedAngle2;
      }

      float stroke1, stroke2;
      float interval1, interval2;

      switch (roundedAngle1)
      {
      default: 0.0;
      }

      stroke1 -= (roundedAngle1 - _angle1) * interval1;

      switch (roundedAngle2)
      {
      default: 0.0;
      }

      stroke2 -= (roundedAngle2 - _angle2) * interval2;

    };

  }
}

#endif
