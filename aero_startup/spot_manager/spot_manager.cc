#include <ros/ros.h>
#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <yaml-cpp/yaml.h>

#include <seed_mover/SaveSpot.h>
#include <seed_mover/GetSpot.h>

class SpotManager{
public:
  SpotManager(ros::NodeHandle _nh, std::string _file);
  ~SpotManager();

  bool SaveSpot(seed_mover::SaveSpot::Request &req,
		seed_mover::SaveSpot::Response &res);
  bool GetSpot(seed_mover::GetSpot::Request &req,
	       seed_mover::GetSpot::Response &res);
  void DeleteSpot();
  void GetList();

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  ros::ServiceServer save_spot_;
  ros::ServiceServer get_spot_;

  std::string file_;
};

SpotManager::SpotManager(ros::NodeHandle _nh, std::string _file)
  : nh_(_nh),
    file_(_file)
{
  save_spot_ = nh_.advertiseService("save_spot", &SpotManager::SaveSpot, this);
  get_spot_ = nh_.advertiseService("get_spot", &SpotManager::GetSpot, this);
}

SpotManager::~SpotManager(){};

bool SpotManager::SaveSpot(seed_mover::SaveSpot::Request &req,
			   seed_mover::SaveSpot::Response &res)
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
  res.status = true;
  return true;
}

bool SpotManager::GetSpot(seed_mover::GetSpot::Request &req,
			  seed_mover::GetSpot::Response &res)
{
  YAML::Node config = YAML::LoadFile(file_);
  bool found = false;
  int index = 0;
  for (int i=0; i < config.size(); ++i) {
    if (config[i]["spot"]["name"].as<std::string>() == req.name){
      index = i;
      found = true;
      break;
    }
  }
  if (!found) {
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
