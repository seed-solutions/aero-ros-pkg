#include <ros/ros.h>
#include <aero_startup/HandControl.h>
#include <aero_startup/GraspControl.h>

#include <aero_ros_controller/RobotInterface.hh>

/// robot dependant constant
#define POSITION_Right 12
#define POSITION_Left  27

class AeroHandInterface : public robot_interface::RobotInterface
{
public: AeroHandInterface(ros::NodeHandle &_nh) : RobotInterface(_nh)
  {
    rhand.reset(new robot_interface::TrajectoryClient(_nh,
                            "rhand_controller/follow_joint_trajectory",
                            "rhand_controller/state",
                             { "r_indexbase_joint", "r_thumb_joint" }));
    this->add_controller("rhand", rhand);

    lhand.reset(new robot_interface::TrajectoryClient(_nh,
                            "lhand_controller/follow_joint_trajectory",
                            "lhand_controller/state",
                             { "l_indexbase_joint", "l_thumb_joint" }));
    this->add_controller("lhand", lhand);

    controller_group_["both_hands"]  = {"rhand", "lhand"};
  }

  robot_interface::TrajectoryClient::Ptr rhand; // 2
  robot_interface::TrajectoryClient::Ptr lhand; // 2
};

using namespace aero_startup;

class AeroHandControl {
public:
  AeroHandControl (ros::NodeHandle &nh) : executing_flg_left_(true), executing_flg_right_(true)
  {
    hi.reset(new AeroHandInterface(nh));

    service_ = nh.advertiseService("/aero_hand_controller",
                                   &AeroHandControl::HandControl, this);

    g_client_ = nh.serviceClient<aero_startup::GraspControl>(
      "/aero_controller/grasp_control");
  }

  bool HandControl(aero_startup::HandControl::Request &req,
                   aero_startup::HandControl::Response &res)
  {
    ROS_DEBUG("HandControl com: %d, hand %d, pow: %d, thre: %f %f, lr_ang: %f %f",
              req.command, req.hand, req.power,
              thre_fail, thre_warn, larm_angle, rarm_angle);

    aero_startup::GraspControl g_srv;

    int power = req.power;

    switch (req.command) {
    case HandControlRequest::COMMAND_GRASP:
      {
        // because grasp is really really slow, first grasp-angle
        GraspAngle(req.hand, 0.0, 0.0);
        if (req.hand == HandControlRequest::HAND_LEFT) {
          g_srv.request.position = POSITION_Left;
          g_srv.request.script = GraspControlRequest::SCRIPT_GRASP;
          executing_flg_left_ = true; //executing_grasp_script
        } else if (req.hand == HandControlRequest::HAND_RIGHT) {
          g_srv.request.position = POSITION_Right;
          g_srv.request.script = GraspControlRequest::SCRIPT_GRASP;
          executing_flg_right_ = true; //executing_grasp_script
        } else {
          ROS_ERROR("Unexpected hand %d", req.hand);
          return false;
        }

        if (power != 0) {
          g_srv.request.power = (power << 8) + 30;
        } else {
          g_srv.request.power = (100 << 8) + 30;
        }
        ROS_DEBUG("call pos: %d, script: %d, power %d",
                  g_srv.position, g_srv.script, g_srv.power);
        g_client_.call(g_srv);
        {
          robot_interface::joint_angle_map act_map;
          hi->getActualPositions(act_map);
          ROS_WARN("get act");
          double pos0, pos1;
          if (act_map.find("l_thumb_joint") != act_map.end()) {
            pos0 =  act_map["l_thumb_joint"];
          } else {
            ROS_ERROR("not found 0");
            return false;
          }
          if (act_map.find("r_thumb_joint") != act_map.end()) {
            pos1 =  act_map["r_thumb_joint"];
          } else {
            ROS_ERROR("not found 1");
            return false;
          }
          if(g_srv.response.angles.size() < 2) {
            g_srv.response.angles.resize(2);
          }
          g_srv.response.angles[0] = pos0;
          g_srv.response.angles[1] = pos1;
        }

        res.status = true;
        std::string status_msg = "grasp success";
        if (req.hand == HandControlRequest::HAND_LEFT) {
          if (g_srv.response.angles[0] > -req.thre_warn) {
            ROS_DEBUG("bad %f > %f", g_srv.response.angles[0], -req.thre_warn);
            res.success = false;
            status_msg = "grasp bad";
          }
          if (g_srv.response.angles[0] < -req.thre_fail) {
            ROS_DEBUG("failed %f < %f", g_srv.response.angles[0], -req.thre_fail);
            res.success = false;
            status_msg = "grasp failed";
          }
        } else if (req.hand == HandControlRequest::HAND_RIGHT) {
          if (g_srv.response.angles[1] < req.thre_warn) {
            ROS_DEBUG("bad %f < %f", g_srv.response.angles[1], req.thre_fail);
            res.success = false;
            status_msg = "grasp bad";
          }
          if (g_srv.response.angles[1] > req.thre_fail) {
            ROS_DEBUG("faied %f > %f", g_srv.response.angles[1], req.thre_fail);
            res.success = false;
            status_msg = "grasp failed";
          }
        } else {
          ROS_ERROR("Unexpected hand %d", req.hand);
          return false;
        }
        res.status = status_msg;
      }
      break;
    case HandControlRequest::COMMAND_UNGRASP:
      {
        OpenHand(req.hand);
        res.status = true;
        res.status = "ungrasp success";
      }
      break;
    case HandControlRequest::COMMAND_GRASP_ANGLE:
      {
        GraspAngle(req.hand, req.larm_angle, req.rarm_angle);
        res.status = true;
        res.status = "grasp-angle success";
      }
      break;
    case HandControlRequest::COMMAND_GRASP_FAST:
      {
        // grasp till angle, check if grasp is okay, then hold
        //auto joints = GraspAngle(req.hand, 0.0, 0.0, 1.2); // grasp takes about 1 sec
        GraspAngle(req.hand, 0.0, 0.0, 1.2); // grasp takes about 1 sec
        ROS_WARN("end_grasp");
        if (req.hand == HandControlRequest::HAND_LEFT) {
          robot_interface::joint_angle_map act_map;
          hi->getActualPositions(act_map);
          auto it = act_map.find("l_indexbase_joint");
          if (it == act_map.end()) {
            // never occuer
            ROS_ERROR("can not find \"l_indexbase_joint\"");
            return false;
          }
          double pos = act_map["l_indexbase_joint"];
          if (fabs(pos) < 0.05) {
            ROS_DEBUG("fail fabs(pos) = %f < 0.05", pos);
            if (fabs(req.larm_angle) < 0.05) {
              ROS_DEBUG("fail: OpenHand");
              OpenHand(req.hand);
            } else {
              ROS_DEBUG("fail: GraspAngle");
              GraspAngle(req.hand, req.larm_angle, 0.0, 1.2);
            }
            res.success = false;
            res.status = "grasp failed";
            return true;
          }
          g_srv.request.position = POSITION_Left;
          g_srv.request.script = GraspControlRequest::SCRIPT_GRASP;
          executing_flg_left_ = true; //executing_grasp_script
        } else if (req.hand == HandControlRequest::HAND_RIGHT) {
          robot_interface::joint_angle_map act_map;
          hi->getActualPositions(act_map);
          auto it = act_map.find("r_indexbase_joint");
          if (it == act_map.end()) {
            // never occuer
            ROS_ERROR("can not find \"r_indexbase_joint\"");
            return false;
          }
          double pos = act_map["r_indexbase_joint"];
          if (fabs(pos) < 0.05) {
            ROS_DEBUG("fail fabs(pos) = %f < 0.05", pos);
            if (fabs(req.rarm_angle) < 0.05) {
              ROS_DEBUG("fail: OpenHand");
              OpenHand(req.hand);
            } else {
              ROS_DEBUG("fail: GraspAngle");
              GraspAngle(req.hand, 0.0, req.rarm_angle, 1.2);
            }
            res.success = true;
            res.status = "grasp failed";
            return true;
          }
          g_srv.request.position = POSITION_Right;
          g_srv.request.script = GraspControlRequest::SCRIPT_GRASP;
          executing_flg_right_ = true; //executing_grasp_script
        }

        if (power != 0) {
          g_srv.request.power = (power << 8) + 30;
        } else {
          g_srv.request.power = (100 << 8) + 30;
        }
        ROS_DEBUG("call pos: %d, script: %d, power %d",
                  g_srv.position, g_srv.script, g_srv.power);
        g_client_.call(g_srv);
        {
          robot_interface::joint_angle_map act_map;
          hi->getActualPositions(act_map);
          double pos0, pos1;
          if (act_map.find("l_thumb_joint") != act_map.end()) {
            pos0 =  act_map["l_thumb_joint"];
          } else {
            return false;
          }
          if (act_map.find("r_thumb_joint") != act_map.end()) {
            pos1 =  act_map["r_thumb_joint"];
          } else {
            return false;
          }
          if(g_srv.response.angles.size() < 2) {
            g_srv.response.angles.resize(2);
          }
          g_srv.response.angles[0] = pos0;
          g_srv.response.angles[1] = pos1;
        }

        res.success = true;
        std::string status_msg = "grasp success";
        if (req.hand == HandControlRequest::HAND_LEFT) {
          if (g_srv.response.angles[0] > req.thre_fail) {
            ROS_DEBUG("faied %f > %f", g_srv.response.angles[0], req.thre_fail);
            res.success = false;
            status_msg = "grasp failed";
          }
        } else if (req.hand == HandControlRequest::HAND_RIGHT) {
          if (g_srv.response.angles[1] < -req.thre_fail) {
            ROS_DEBUG("faied %f < %f", g_srv.response.angles[1], -req.thre_fail);
            res.success = false;
            status_msg = "grasp failed";
          }
        }
        res.status = status_msg;
      }
      break;
    }
    return true;
  }

  void OpenHand(int hand)
  {
    ROS_DEBUG("OpenHand %d", hand);
    aero_startup::GraspControl g_srv;

    robot_interface::joint_angle_map map;
    if (hand == HandControlRequest::HAND_LEFT) {
      g_srv.request.position = POSITION_Left;
      g_srv.request.script = GraspControlRequest::SCRIPT_UNGRASP;
      executing_flg_left_ = false;

      map["l_thumb_joint"] = -15.0 * M_PI / 180.0;
    } else if (hand == HandControlRequest::HAND_RIGHT) {
      g_srv.request.position = POSITION_Right;
      g_srv.request.script = GraspControlRequest::SCRIPT_UNGRASP;
      executing_flg_right_ = false;

      map["r_thumb_joint"] =  15.0 * M_PI / 180.0;
    }
    //
    ros::Time start = ros::Time::now() + ros::Duration(0.04);
    ROS_DEBUG("OpenHand: sendAngles");
    hi->sendAngles(map, 0.5, start);
    ROS_DEBUG("OpanHand: wait_interpolation");
    hi->wait_interpolation();
    //
    g_srv.request.power = (100 << 8) + 30;
    ROS_DEBUG("call pos: %d, script: %d, power %d",
              g_srv.position, g_srv.script, g_srv.power);
    g_client_.call(g_srv);
  }

  void GraspAngle (int hand, float larm_angle, float rarm_angle, float time=0.5)
  {
    ROS_DEBUG("Grasp Angle: %d %f %f %f", hand, larm_angle, rarm_angle, time);
    //aero_startup::AeroSendJoints srv;
    robot_interface::joint_angle_map map;
    switch(hand) {
    case HandControlRequest::HAND_BOTH:
      map["l_indexbase_joint"]  = -larm_angle * M_PI / 180;
      map["l_thumb_joint"]      =  larm_angle * M_PI / 180/ 4.0;
      map["r_indexbase_joint"]  = -rarm_angle * M_PI / 180;
      map["r_thumb_joint"]      =  rarm_angle * M_PI / 180 / 4.0;
#if 0 // trx_s
      srv.request.joint_names = {"l_thumb_joint", "r_thumb_joint"};
      srv.request.points.positions.resize(2);
      srv.request.points.positions[0] = larm_angle * M_PI / 180;
      srv.request.points.positions[1] = rarm_angle * M_PI / 180;
#endif
      break;
    case HandControlRequest::HAND_LEFT:
      if (executing_flg_left_) {
        aero_startup::GraspControl g_srv;
        g_srv.request.position = POSITION_Left;
        g_srv.request.script = GraspControlRequest::SCRIPT_CANCEL;
        g_srv.request.power = (100 << 8) + 30;
        ROS_DEBUG("call pos: %d, script: %d, power %d",
                  g_srv.position, g_srv.script, g_srv.power);
        g_client_.call(g_srv);
        executing_flg_left_ = false;
      }
      map["l_indexbase_joint"]  = -larm_angle * M_PI / 180;
      map["l_thumb_joint"]      =  larm_angle * M_PI / 180/ 4.0;
#if 0 // trx_s
      srv.request.joint_names = {"l_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = larm_angle * M_PI / 180;
#endif
      break;
    case HandControlRequest::HAND_RIGHT:
      if (executing_flg_right_) {
        aero_startup::GraspControl g_srv;
        g_srv.request.position = POSITION_Right;
        g_srv.request.script = GraspControlRequest::SCRIPT_CANCEL;
        g_srv.request.power = (100 << 8) + 30;
        ROS_DEBUG("call pos: %d, script: %d, power %d",
                  g_srv.position, g_srv.script, g_srv.power);
        g_client_.call(g_srv);
        executing_flg_right_ = 0;
      }
      map["r_indexbase_joint"]  = -rarm_angle * M_PI / 180;
      map["r_thumb_joint"]      =  rarm_angle * M_PI / 180 / 4.0;
#if 0
      srv.request.joint_names = {"r_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = rarm_angle * M_PI / 180;
#endif
      break;
    }
    ROS_DEBUG("GraspAngle: sendAngles");
    ros::Time start = ros::Time::now() + ros::Duration(0.04);
    hi->sendAngles(map, time, start);
    ROS_DEBUG("GraspAngle: wait_interpolation");
    hi->wait_interpolation();
  }

private:
  bool executing_flg_left_;
  bool executing_flg_right_;

  ros::ServiceClient g_client_;
  ros::ServiceServer service_;

  boost::shared_ptr<AeroHandInterface > hi;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_hand_controller");
  ros::NodeHandle nh;
  AeroHandControl ahc(nh);

  ros::spin();

  return 0;
}
