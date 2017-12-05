namespace aero
{
  namespace common
  {

    float TableTemplate (float _stroke)
    {
      int roundedStroke = static_cast<int>(_stroke);
      int roundedStrokeIndex = roundedStroke - ArrayTableTemplateOffset;
      if(static_cast<int>(TableTemplateCandidates.size() - 1) < roundedStrokeIndex) roundedStrokeIndex = static_cast<int>(TableTemplateCandidates.size() - 1); 
      if(roundedStrokeIndex < 0) roundedStrokeIndex = 0;
      auto ref = TableTemplateCandidates.at(roundedStrokeIndex);
      std::vector<S2AData> candidates = ref.first;
      std::vector<S2AData> appendix = ref.second;

      if (_stroke < 0) {
        if (candidates.size() >= 2)
          if (candidates[0].stroke < candidates[1].stroke)
            std::reverse(candidates.begin(), candidates.end());

        for (unsigned int i = 0; i < candidates.size(); ++i)
          if (_stroke >= candidates[i].stroke)
            if (candidates[i].range == 0)
              return candidates[i].angle;
            else
              return candidates[i].angle
                - (candidates[i].stroke - _stroke) / candidates[i].range;

        if (appendix.size() >= 2)
          if (appendix[0].stroke < appendix[1].stroke)
            std::reverse(appendix.begin(), appendix.end());

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
          if (appendix[0].range == 0)
            return appendix[0].angle;
          else
            return appendix[0].angle
              - (appendix[0].stroke - _stroke) / appendix[0].range;
        }
      }

      else {
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

        if (appendix.size() == 0) {
          return candidates[candidates.size() - 1].angle;
        } else {
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
