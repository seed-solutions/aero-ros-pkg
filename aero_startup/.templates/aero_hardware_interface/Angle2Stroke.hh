namespace aero
{
  namespace common
  {

    float TableTemplate (float _angle)
    {
      int roundedAngle = static_cast<int>(_angle);
      if (_angle > roundedAngle + 0.001) ++roundedAngle;
      int roundedAngleIndex = roundedAngle - ArrayTableTemplateOffset;
      auto ref = TableTemplateMap.at(roundedAngleIndex);
      float stroke = ref.first;
      float interval = ref.second;

      return stroke - (roundedAngle - _angle) * interval;
    };

    dualJoint TableTemplate (float _angle1, float _angle2)
    {
      int roundedAngle1 = static_cast<int>(_angle1);
      if (_angle1 < 0) {
        if (_angle1 < roundedAngle1 - 0.001) --roundedAngle1;
      } else {
        if (_angle1 > roundedAngle1 + 0.001) ++roundedAngle1;
      }

      int roundedAngle2 = static_cast<int>(_angle2);
      if (_angle2 < 0) {
        if (_angle2 < roundedAngle2 - 0.001) --roundedAngle2;
      } else {
        if (_angle2 > roundedAngle2 + 0.001) ++roundedAngle2;
      }

      int roundedAngleIndex1 = (roundedAngle1 > 0 ? roundedAngle1 : -roundedAngle1 + ArrayTableTemplateNegativeOffset1);
      int roundedAngleIndex2 = (roundedAngle2 > 0 ? roundedAngle2 : -roundedAngle2 + ArrayTableTemplateNegativeOffset2);
      auto ref1 = TableTemplateMap1.at(roundedAngleIndex1);
      auto ref2 = TableTemplateMap2.at(roundedAngleIndex2);
      float stroke1 = ref1.first, stroke2 = ref2.first;
      float interval1 = ref1.second, interval2 = ref2.second;

      stroke1 -= (roundedAngle1 - _angle1) * interval1;
      stroke2 -= (roundedAngle2 - _angle2) * interval2;

      return {stroke2 + stroke1, stroke2 - stroke1} ;
    };

  }
}
