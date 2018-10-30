#ifndef _AERO_ROBOT_INTERFACE_
#define _AERO_ROBOT_INTERFACE_

#include <aero_ros_controller/RobotInterface.hh>

namespace aero {

class AeroRobotInterface : public robot_interface::RobotInterface {
public:
  typedef boost::shared_ptr<AeroRobotInterface > Ptr;

public:
  AeroRobotInterface(ros::NodeHandle &_nh) : robot_interface::RobotInterface(_nh) {
    //
    configureFromParam("robot_interface_controllers");
#define ADD_CONTROLLER(limb)                    \
    {                                           \
      auto it = controllers_.find(#limb);       \
      if (it != controllers_.end()) {           \
        this-> limb = it->second;               \
      }                                         \
    }

    ADD_CONTROLLER(rarm);
    ADD_CONTROLLER(larm);
    ADD_CONTROLLER(waist);
    ADD_CONTROLLER(lifter);
    ADD_CONTROLLER(head);

    // group settings
    add_group("both_arms",   {"rarm", "larm"});
    add_group("upper_body",  {"rarm", "larm", "waist", "head"});
    add_group("torso",       {"waist", "lifter"});
    add_group("without_head",{"rarm", "larm", "waist", "lifter"});

    for(int i = 0; i < joint_list_.size(); i++) {
      ROS_INFO("j%d: %s", i, joint_list_[i].c_str());
    }
  }

  AeroRobotInterface(ros::NodeHandle &_nh, bool _only_head) : robot_interface::RobotInterface(_nh) {
    if (_only_head) {
      // head
      head.reset(new robot_interface::TrajectoryClient(_nh,
                                      "head_controller/follow_joint_trajectory",
                                      "head_controller/state",
                                      { "neck_y_joint", "neck_p_joint", "neck_r_joint"}
                                      ));
      this->add_controller("head",   head);
    }
  }

  bool sendAngles_wo_head(const std::vector < std::string> &_names,
                          const std::vector< double> &_positions,
                          const double _tm, const ros::Time &_start)
  {
    for(auto it = controllers_.begin(); it != controllers_.end(); it++) {
      if(it->first != "head") {
        it->second->sendAngles(_names, _positions, _tm, _start);
      }
    }
  }

public:
  robot_interface::TrajectoryClient::Ptr larm; // 8
  robot_interface::TrajectoryClient::Ptr rarm; // 8
  robot_interface::TrajectoryClient::Ptr waist; // 3
  robot_interface::TrajectoryClient::Ptr lifter;// 2
  robot_interface::TrajectoryClient::Ptr head;  // 3 // 24 jsk / 22 thk
  // TrajectoryClient::Ptr l_hand;
  // TrajectoryClient::Ptr r_hand;
};

}
#endif
