/// @author Shintaro Hori

#include <ros/ros.h>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include <aero_std/SaveSpot.h>
#include <aero_std/GetSpot.h>
#include <aero_std/DeleteSpot.h>
#include <aero_std/GetSpots.h>

namespace aero {
struct Spot {
  std::string name;
  geometry_msgs::Pose pose;
};

class SpotManager {
 public:
  SpotManager(ros::NodeHandle _nh, std::string _file);
  ~SpotManager();

  bool SaveSpot(aero_std::SaveSpot::Request & req,
                aero_std::SaveSpot::Response& res);
  bool GetSpot(aero_std::GetSpot::Request & req,
               aero_std::GetSpot::Response& res);
  bool DeleteSpot(aero_std::DeleteSpot::Request & req,
                  aero_std::DeleteSpot::Response& res);
  bool GetSpots(aero_std::GetSpots::Request & req,
                aero_std::GetSpots::Response& res);
  void BroadcastSpots(const ros::TimerEvent& ev);

 private:
  int GetIndex_(std::string _name);
  std::vector<std::string> GetList_();

  void ReadFromFile_();
  void WriteIntoFile_();
  std::vector<Spot> spots_;

  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;
  ros::ServiceServer save_spot_;
  ros::ServiceServer get_spot_;
  ros::ServiceServer delete_spot_;
  ros::ServiceServer get_spots_;
  ros::Timer timer_;

  std::string file_;
};

/// @brief Constructor
/// @param _nh NodeHandle
/// @param _file database file (yaml)
SpotManager::SpotManager(ros::NodeHandle _nh, std::string _file)
  : nh_(_nh),
    file_(_file) {
  ReadFromFile_();
  save_spot_ = nh_.advertiseService("save_spot",
                                    &SpotManager::SaveSpot,
                                    this);
  get_spot_ = nh_.advertiseService("get_spot",
                                   &SpotManager::GetSpot,
                                   this);
  delete_spot_ = nh_.advertiseService("delete_spot",
                                      &SpotManager::DeleteSpot,
                                      this);
  get_spots_ = nh_.advertiseService("get_spots",
                                    &SpotManager::GetSpots,
                                    this);
  timer_ = nh_.createTimer(ros::Duration(0.1),
                           &SpotManager::BroadcastSpots, this);
}

SpotManager::~SpotManager() {
}

/// @brief save current base_link as spot
/// @param req spot name (req.name)
/// @param res return status
bool SpotManager::SaveSpot(aero_std::SaveSpot::Request & req,
                           aero_std::SaveSpot::Response& res) {
  tf::StampedTransform tr;
  try {
    listener_.lookupTransform("/map", "/base_link", ros::Time(0), tr);
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    res.status = false;
    return false;
  }

  int index = GetIndex_(req.name);

  if (index < 0) {
    ROS_INFO("add new location : %s", req.name.c_str());
    Spot new_spot;
    index = spots_.size();
    spots_.push_back(new_spot);
  } else {
    ROS_INFO("renew location : %s", req.name.c_str());
  }

  Spot& spot = spots_[index];
  spot.name = req.name;
  spot.pose.position.x = tr.getOrigin().x();
  spot.pose.position.y = tr.getOrigin().y();
  spot.pose.position.z = tr.getOrigin().z();
  spot.pose.orientation.x = tr.getRotation().x();
  spot.pose.orientation.y = tr.getRotation().y();
  spot.pose.orientation.z = tr.getRotation().z();
  spot.pose.orientation.w = tr.getRotation().w();
  WriteIntoFile_();

  res.status = true;
  return true;
}

/// @brief get spot pose
/// @param req spot name (req.name)
/// @param res spot pose (res.pose)
bool SpotManager::GetSpot(aero_std::GetSpot::Request & req,
                          aero_std::GetSpot::Response& res) {
  int index = GetIndex_(req.name);

  if (index < 0) {
    res.status = false;
    res.error = "such spot is not found";
    return false;
  }

  res.status = true;
  res.pose = spots_[index].pose;

  return true;
}

/// @brief delete designated spot
/// @param req spot name (req.name)
/// @param res return status
bool SpotManager::DeleteSpot(aero_std::DeleteSpot::Request & req,
                             aero_std::DeleteSpot::Response& res) {
  int index = GetIndex_(req.name);

  if (index < 0) {
    ROS_WARN("spot: %s doesn't exist", req.name.c_str());
    res.status = false;
    return false;
  }

  std::vector<Spot> prev_spots;
  prev_spots.assign(spots_.begin(), spots_.end());
  spots_.clear();

  for (int i = 0; i < prev_spots.size(); ++i) {
    if (i != index) {
      spots_.push_back(prev_spots[i]);
    }
  }

  WriteIntoFile_();

  ROS_INFO("spot: %s is successfully deleted", req.name.c_str());
  res.status = true;
  return true;
}

/// @berief get all spot names
/// @param req none
/// @param res spot name list (res.spots)
bool SpotManager::GetSpots(aero_std::GetSpots::Request & req,
                           aero_std::GetSpots::Response& res) {
  std::vector<std::string> list = GetList_();
  for (auto it: list) res.spots.push_back(it);
  return true;
}

/// @brief read spots from yaml file
void SpotManager::ReadFromFile_() {
  spots_.clear();
  std::ifstream ifs(file_);

  if (ifs.is_open()) {
    YAML::Node config = YAML::LoadFile(file_);
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
void SpotManager::WriteIntoFile_() {
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
  std::ofstream ofs(file_, std::ios_base::trunc);
  ofs << config;
  ofs << std::endl;
}

/// @brief get index of spot
/// @param _name spot name
/// @return spot index, -1 if not found
int SpotManager::GetIndex_(std::string _name) {
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
std::vector<std::string> SpotManager::GetList_() {
  std::vector<std::string> res;
  for (auto it: spots_) {
    res.push_back(it.name);
  }
  return res;
}

void SpotManager::BroadcastSpots(const ros::TimerEvent& ev)
{
  for (auto spot: spots_) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(spot.pose.position.x,
                                    spot.pose.position.y,
                                    spot.pose.position.z));
    transform.setRotation(tf::Quaternion(spot.pose.orientation.x,
                                         spot.pose.orientation.y,
                                         spot.pose.orientation.z,
                                         spot.pose.orientation.w));
    broadcaster_.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "map", spot.name));
  }
}
} // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "spot_manager");
  ros::NodeHandle nh;

  std::string file;
  ros::param::get("/spot_manager/file", file);
  ROS_INFO("save to %s", file.c_str());
  aero::SpotManager sm(nh, file);

  ros::spin();

  return 0;
}
