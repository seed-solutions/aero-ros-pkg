#ifndef _AERO_STD_PARSE_
#define _AERO_STD_PARSE_

#include <string>
#include <fstream>

namespace aero
{
  namespace parse
  {
    inline std::string strparseLine(std::ifstream &ifs, std::string &line) {
      std::getline(ifs, line);
      auto p = line.find(":");
      return std::string(line.begin() + p + 2, line.end());
    }

    inline int iparseLine(std::ifstream &ifs, std::string &line) {
      std::getline(ifs, line);
      auto p = line.find(":");
      return std::stoi(std::string(line.begin() + p + 2, line.end()));
    }

    inline float fparseLine(std::ifstream &ifs, std::string &line) {
      std::getline(ifs, line);
      auto p = line.find(":");
      return std::stof(std::string(line.begin() + p + 2, line.end()));
    }
  }
}

#endif
