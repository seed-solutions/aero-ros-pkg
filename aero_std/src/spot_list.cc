/// @author Shintaro Hori, Hiroaki Yaguchi JSK

#include <aero_std/spot_list.hh>

namespace aero {

/// @brief make Spot from args
/// @param name spot name
/// @param tx position x
/// @param ty position y
/// @param tz position z
/// @param qx quaternion x
/// @param qy quaternion y
/// @param qz quaternion z
/// @param qw quaternion w
Spot MakeSpot(std::string name,
              float tx, float ty, float tz,
              float qx, float qy, float qz, float qw) {
  Spot spot;
  spot.name = name;
  spot.pose.position.x = tx;
  spot.pose.position.y = ty;
  spot.pose.position.z = tz;
  spot.pose.orientation.x = qx;
  spot.pose.orientation.y = qy;
  spot.pose.orientation.z = qz;
  spot.pose.orientation.w = qw;

  return spot;
}

SpotList::SpotList() {
}

SpotList::~SpotList() {
}

/// @brief read spots from yaml file
void SpotList::ReadFromFile(std::string& _file) {
  spots_.clear();
  std::ifstream ifs(_file);

  if (ifs.is_open()) {
    YAML::Node config = YAML::LoadFile(_file);
    for (size_t i = 0; i < config.size(); ++i) {
      Spot spot;
      spot.name = config[i]["spot"]["name"].as<std::string>();
      spot.pose.position.x = config[i]["spot"]["position"]["x"].as<float>();
      spot.pose.position.y = config[i]["spot"]["position"]["y"].as<float>();
      spot.pose.position.z = config[i]["spot"]["position"]["z"].as<float>();
      spot.pose.orientation.x =
        config[i]["spot"]["orientation"]["x"].as<float>();
      spot.pose.orientation.y =
        config[i]["spot"]["orientation"]["y"].as<float>();
      spot.pose.orientation.z =
        config[i]["spot"]["orientation"]["z"].as<float>();
      spot.pose.orientation.w =
        config[i]["spot"]["orientation"]["w"].as<float>();
      spots_.push_back(spot);
    }
  }
}

/// @brief write spots into file
void SpotList::WriteIntoFile(std::string& _file) {
  YAML::Node config;
  for (size_t i = 0; i < spots_.size(); ++i) {
    Spot& spot = spots_[i];
    config[i]["spot"]["name"] = spot.name;
    config[i]["spot"]["position"]["x"] = spot.pose.position.x;
    config[i]["spot"]["position"]["y"] = spot.pose.position.y;
    config[i]["spot"]["position"]["z"] = spot.pose.position.z;
    config[i]["spot"]["orientation"]["x"] = spot.pose.orientation.x;
    config[i]["spot"]["orientation"]["y"] = spot.pose.orientation.y;
    config[i]["spot"]["orientation"]["z"] = spot.pose.orientation.z;
    config[i]["spot"]["orientation"]["w"] = spot.pose.orientation.w;
  }
  std::ofstream ofs(_file, std::ios_base::trunc);
  ofs << config;
  ofs << std::endl;
}

/// @brief get index of spot
/// @param _name spot name
/// @return spot index, -1 if not found
int SpotList::GetIndex(std::string& _name) {
  int result = -1;
  int index = 0;

  for (auto it: spots_) {
    if (_name == it.name) {
      result = index;
      break;
    }
    ++index;
  }
  return result;
}

/// @brief get all spot names
/// @return spot name list
std::vector<std::string> SpotList::GetList() {
  std::vector<std::string> res;
  for (auto it: spots_) {
    res.push_back(it.name);
  }
  return res;
}

/// @brief save spot, if exist spot name overwrite, else add
/// @param _spot spot
void SpotList::SaveSpot(Spot& _spot) {
  int index = GetIndex(_spot.name);

  if (index < 0) {
    AddSpot(_spot);
  } else {
    UpdateSpot(index, _spot);
  }
}

/// @brief update spot
/// @param _index index
/// @param _spot spot
void SpotList::UpdateSpot(int _index, Spot& _spot) {
  spots_[_index] = _spot;
}

/// @brief add new spot
/// @param _spot spot
void SpotList::AddSpot(Spot& _spot) {
  spots_.push_back(_spot);
}

/// @brief delete designated spot
/// @param req spot name
/// @return false if not exist
bool SpotList::DeleteSpot(std::string& _name) {
  int index = GetIndex(_name);
  if (index < 0) {
    return false;
  } else {
    std::vector<Spot> prev_spots;
    prev_spots.assign(spots_.begin(), spots_.end());
    spots_.clear();

    for (int i = 0; i < prev_spots.size(); ++i) {
      if (i != index) {
        spots_.push_back(prev_spots[i]);
      }
    }
  }
  return true;
}

}  // namespace
