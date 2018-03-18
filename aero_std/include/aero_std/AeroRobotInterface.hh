#ifndef _AERO_ROBOT_INTERFACE_
#define _AERO_ROBOT_INTERFACE_

#include <aero_ros_controller/RobotInterface.hh>

namespace aero {

class AeroRobotInterface : public robot_interface::RobotInterface {
public:
  typedef boost::shared_ptr<AeroRobotInterface > Ptr;

public:
  AeroRobotInterface(ros::NodeHandle &_nh) : robot_interface::RobotInterface(_nh) {
    // rarm
    rarm.reset(new robot_interface::TrajectoryClient(_nh,
                                    "rarm_controller/follow_joint_trajectory",
                                    "rarm_controller/state",
                                    { "r_shoulder_p_joint", "r_shoulder_r_joint", "r_shoulder_y_joint",
                                        "r_elbow_joint", "r_wrist_y_joint", "r_wrist_p_joint", "r_wrist_r_joint",
                                        "r_hand_y_joint", // jsk
                                        }
                                    ));
    this->add_controller("rarm", rarm);

    // larm
    larm.reset(new robot_interface::TrajectoryClient(_nh,
                                    "larm_controller/follow_joint_trajectory",
                                    "larm_controller/state",
                                    { "l_shoulder_p_joint", "l_shoulder_r_joint", "l_shoulder_y_joint",
                                        "l_elbow_joint", "l_wrist_y_joint", "l_wrist_p_joint", "l_wrist_r_joint",
                                        "l_hand_y_joint", // jsk
                                        }
                                    ));
    this->add_controller("larm", larm);

    // torso
    waist.reset(new robot_interface::TrajectoryClient(_nh,
                                     "waist_controller/follow_joint_trajectory",
                                     "waist_controller/state",
                                     { "waist_y_joint", "waist_p_joint", "waist_r_joint"}
                                     ));
    this->add_controller("waist",  waist);

    // lifter
    lifter.reset(new robot_interface::TrajectoryClient(_nh,
                                      "lifter_controller/follow_joint_trajectory",
                                      "lifter_controller/state",
                                      { "knee_joint", "ankle_joint" }
                                      ));
    this->add_controller("lifter", lifter);

    // head
    head.reset(new robot_interface::TrajectoryClient(_nh,
                                    "head_controller/follow_joint_trajectory",
                                    "head_controller/state",
                                    { "neck_y_joint", "neck_p_joint", "neck_r_joint"}
                                    ));
    this->add_controller("head",   head);

    // group settings
    controller_group_["both_arms"]  = {"rarm", "larm"};
    controller_group_["upper_body"] = {"rarm", "larm", "waist", "head"};
    controller_group_["torso"]      = {"waist", "lifter"};
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
