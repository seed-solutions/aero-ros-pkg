/// @author Shintaro Hori

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <aero_std/spot_list.hh>

#include <aero_std/SaveSpot.h>
#include <aero_std/GetSpot.h>
#include <aero_std/DeleteSpot.h>
#include <aero_std/GetSpots.h>

namespace aero {
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
  SpotList spot_list_;

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
  spot_list_.ReadFromFile(file_);
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

  Spot spot;
  spot.name = req.name;
  spot.pose.position.x = tr.getOrigin().x();
  spot.pose.position.y = tr.getOrigin().y();
  spot.pose.position.z = tr.getOrigin().z();
  spot.pose.orientation.x = tr.getRotation().x();
  spot.pose.orientation.y = tr.getRotation().y();
  spot.pose.orientation.z = tr.getRotation().z();
  spot.pose.orientation.w = tr.getRotation().w();

  spot_list_.SaveSpot(spot);
  spot_list_.WriteIntoFile(file_);

  res.status = true;
  return true;
}

/// @brief get spot pose
/// @param req spot name (req.name)
/// @param res spot pose (res.pose)
bool SpotManager::GetSpot(aero_std::GetSpot::Request & req,
                          aero_std::GetSpot::Response& res) {
  int index = spot_list_.GetIndex(req.name);

  if (index < 0) {
    res.status = false;
    res.error = "such spot is not found";
    return false;
  }

  res.status = true;
  res.pose = spot_list_.spots()[index].pose;

  return true;
}

/// @brief delete designated spot
/// @param req spot name (req.name)
/// @param res return status
bool SpotManager::DeleteSpot(aero_std::DeleteSpot::Request & req,
                             aero_std::DeleteSpot::Response& res) {
  bool result = spot_list_.DeleteSpot(req.name);

  if (!result) {
    ROS_WARN("spot: %s doesn't exist", req.name.c_str());
    res.status = false;
    return false;
  }

  spot_list_.WriteIntoFile(file_);

  ROS_INFO("spot: %s is successfully deleted", req.name.c_str());
  res.status = true;
  return true;
}

/// @berief get all spot names
/// @param req none
/// @param res spot name list (res.spots)
bool SpotManager::GetSpots(aero_std::GetSpots::Request & req,
                           aero_std::GetSpots::Response& res) {
  std::vector<std::string> list = spot_list_.GetList();
  for (auto it: list) res.spots.push_back(it);
  return true;
}

/// @brief broadcast all spot as tf
void SpotManager::BroadcastSpots(const ros::TimerEvent& ev)
{
  for (auto spot: spot_list_.spots()) {
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
