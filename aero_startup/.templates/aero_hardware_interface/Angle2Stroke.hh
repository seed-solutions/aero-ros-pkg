namespace aero
{
  namespace common
  {

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

      return {stroke2 + stroke1, stroke2 - stroke1} ;
    };

  }
}
