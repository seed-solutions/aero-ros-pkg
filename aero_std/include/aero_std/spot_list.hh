/// @author Shintaro Hori, Hiroaki Yaguchi JSK

#ifndef AERO_STD_SPOT_LIST_HH_
#define AERO_STD_SPOT_LIST_HH_

#include <iostream>
#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include <geometry_msgs/Pose.h>

namespace aero {
struct Spot {
  std::string name;
  geometry_msgs::Pose pose;
};

Spot MakeSpot(std::string name,
              float tx, float ty, float tz,
              float qx, float qy, float qz, float qw);

class SpotList {
public:
  SpotList();
  ~SpotList();

  int GetIndex(std::string& _name);
  std::vector<std::string> GetList();
  void SaveSpot(Spot& _spot);
  void UpdateSpot(int _index, Spot& _spot);
  void AddSpot(Spot& _spot);
  bool DeleteSpot(std::string& _name);

  void ReadFromFile(std::string& _file);
  void WriteIntoFile(std::string& _file);

  std::vector<Spot>& spots() {return spots_;}

private:
  std::vector<Spot> spots_;
};
}  // namespace

#endif  // AERO_STD_SPOT_LIST_HH_
