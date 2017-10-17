#include <ros/ros.h>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include <aero_std/SaveSpot.h>
#include <aero_std/GetSpot.h>
#include <aero_std/DeleteSpot.h>
#include <aero_std/GetSpots.h>

class SpotManager{
public:
  SpotManager(ros::NodeHandle _nh, std::string _file);
  ~SpotManager();

  bool SaveSpot(aero_std::SaveSpot::Request &req,
		aero_std::SaveSpot::Response &res);
  bool GetSpot(aero_std::GetSpot::Request &req,
	       aero_std::GetSpot::Response &res);
  bool DeleteSpot(aero_std::DeleteSpot::Request &req,
	       aero_std::DeleteSpot::Response &res);
  bool GetSpots(aero_std::GetSpots::Request &req,
                aero_std::GetSpots::Response &res);

private:
  int GetIndex_(std::string _name);
  std::vector<std::string> GetList_();
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  ros::ServiceServer save_spot_;
  ros::ServiceServer get_spot_;
  ros::ServiceServer delete_spot_;
  ros::ServiceServer get_spots_;

  std::string file_;
};

SpotManager::SpotManager(ros::NodeHandle _nh, std::string _file)
  : nh_(_nh),
    file_(_file)
{
  save_spot_ = nh_.advertiseService("save_spot", &SpotManager::SaveSpot, this);
  get_spot_ = nh_.advertiseService("get_spot", &SpotManager::GetSpot, this);
  delete_spot_ = nh_.advertiseService("delete_spot", &SpotManager::DeleteSpot, this);
  get_spots_ = nh_.advertiseService("get_spots", &SpotManager::GetSpots, this);
}

SpotManager::~SpotManager(){};

bool SpotManager::SaveSpot(aero_std::SaveSpot::Request &req,
			   aero_std::SaveSpot::Response &res)
{
  tf::StampedTransform tr;
  try{
    listener_.lookupTransform("/map", "/base_link",ros::Time(0), tr);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    res.status = false;
    return false;
  }

  int index = GetIndex_(req.name);

  if (index < 0) {
    ROS_INFO("add new location : %s", req.name.c_str());
    std::ofstream ofs(file_, std::ios_base::app);
    ofs << "- spot:"              << std::endl;
    ofs << "        name: "         << req.name << std::endl;
    ofs << "        position:"      << std::endl;
    ofs << "            x: "        << tr.getOrigin().x() << std::endl;
    ofs << "            y: "        << tr.getOrigin().y() << std::endl;
    ofs << "            z: "        << tr.getOrigin().z() << std::endl;
    ofs << "        orientation:"   << std::endl;
    ofs << "            x: "        << tr.getRotation().x() << std::endl;
    ofs << "            y: "        << tr.getRotation().y() << std::endl;
    ofs << "            z: "        << tr.getRotation().z() << std::endl;
    ofs << "            w: "        << tr.getRotation().w() << std::endl;
  } else {
    ROS_INFO("renew location : %s", req.name.c_str());
    YAML::Node config = YAML::LoadFile(file_);
    config[index]["spot"]["name"] = req.name;
    config[index]["spot"]["position"]["x"] = tr.getOrigin().x();
    config[index]["spot"]["position"]["y"] = tr.getOrigin().y();
    config[index]["spot"]["position"]["z"] = tr.getOrigin().z();
    config[index]["spot"]["orientation"]["x"] = tr.getRotation().x();
    config[index]["spot"]["orientation"]["y"] = tr.getRotation().y();
    config[index]["spot"]["orientation"]["z"] = tr.getRotation().z();
    config[index]["spot"]["orientation"]["w"] = tr.getRotation().w();
    std::ofstream ofs_renew(file_, std::ios_base::trunc);
    ofs_renew << config;
    ofs_renew << std::endl;
  }
 res.status = true;
 return true;
}

bool SpotManager::GetSpot(aero_std::GetSpot::Request &req,
			  aero_std::GetSpot::Response &res)
{
  YAML::Node config = YAML::LoadFile(file_);
  int index = GetIndex_(req.name);

  if (index < 0) {
    res.status = false;
    res.error = "such spot is not found";
    return false;
  }

  res.status = true;
  res.pose.position.x = config[index]["spot"]["position"]["x"].as<float>();
  res.pose.position.y = config[index]["spot"]["position"]["y"].as<float>();
  res.pose.position.z = config[index]["spot"]["position"]["z"].as<float>();

  res.pose.orientation.w = config[index]["spot"]["orientation"]["w"].as<float>();
  res.pose.orientation.x = config[index]["spot"]["orientation"]["x"].as<float>();
  res.pose.orientation.y = config[index]["spot"]["orientation"]["y"].as<float>();
  res.pose.orientation.z = config[index]["spot"]["orientation"]["z"].as<float>();

  return true;
}

bool SpotManager::DeleteSpot(aero_std::DeleteSpot::Request &req,
			  aero_std::DeleteSpot::Response &res)
{
  YAML::Node config = YAML::LoadFile(file_);
  int index = GetIndex_(req.name);

  if (index < 0) {
    ROS_WARN("spot: %s doesn't exist", req.name.c_str());
    res.status = false;
    return false;
  }

  YAML::Node result;
  for(int i=0; i < config.size(); ++i) {
    if (i!=index) {
      result.push_back(config[i]);
    }
  }

  //config.remove(index);
  std::ofstream ofs(file_, std::ios_base::trunc);
  ofs << result;
  ofs << std::endl;

  ROS_INFO("spot: %s is successfully deleted", req.name.c_str());
  res.status = true;
  return true;
}

bool SpotManager::GetSpots(aero_std::GetSpots::Request &req,
                           aero_std::GetSpots::Response &res)
{
  std::vector<std::string> list = GetList_();

  for(auto it: list) res.spots.push_back(it);
  return true;
}

int SpotManager::GetIndex_(std::string _name) {
  std::vector<std::string> list = GetList_();
  int result = -1;
  int index = 0;
  for (auto it: list) {
    if (_name == it) {
      result = index;
      break;
    }
    ++index;
  }
  return result;
}

std::vector<std::string> SpotManager::GetList_() {
  std::vector<std::string> res;

  std::ifstream ifs(file_);
  bool exist_file = ifs.is_open();
  if (!exist_file) return res;

  YAML::Node config = YAML::LoadFile(file_);
  for (int i=0; i < config.size(); ++i) {
    res.push_back(config[i]["spot"]["name"].as<std::string>());
  }
  return res;
}

int main(int argc, char **argv)
{
	ros::init(argc,argv,"spot_manager");
	ros::NodeHandle nh;

	std::string file;
	ros::param::get("/spot_manager/file" ,file);
	ROS_INFO("save to %s", file.c_str());
	SpotManager sm(nh, file);

	ros::spin();

	return 0;
}
