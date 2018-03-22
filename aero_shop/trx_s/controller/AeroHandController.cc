#include <ros/ros.h>
#include <aero_startup/HandControl.h>
#include <aero_startup/GraspControl.h>
#include <aero_startup/AeroSendJoints.h>

/// robot dependant constant
#define POSITION_Right 12
#define POSITION_Left  27

using namespace aero_startup;

class AeroHandControl {
public:
  AeroHandControl (ros::NodeHandle &nh) : executing_flg_left_(true), executing_flg_right_(true)
  {
    service_ = nh.advertiseService("/aero_hand_controller",
                                   &AeroHandControl::HandControl, this);

    client_ = nh.serviceClient<aero_startup::AeroSendJoints>(
      "/aero_controller/send_joints");
    g_client_ = nh.serviceClient<aero_startup::GraspControl>(
      "/aero_controller/grasp_control");
  }

  bool HandControl(aero_startup::HandControl::Request &req,
                   aero_startup::HandControl::Response &res)
  {
    aero_startup::GraspControl g_srv;

    if (req.hand != HandControlRequest::HAND_LEFT &&
        req.hand != HandControlRequest::HAND_RIGHT &&
        req.hand != HandControlRequest::HAND_BOTH ) {
      ROS_ERROR("not existing hand (= %d) for aero_startup/HandControl service", req.hand);
      res.success = false;
      res.status  = "invalid hand parameter";
      return true;
    }

    int power = req.power;
    res.success = true; // substitude 'false' if needed

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
        }

        if (power != 0) {
          g_srv.request.power = (power << 8) + 30;
        } else {
          g_srv.request.power = (100 << 8) + 30;
        }

        g_client_.call(g_srv);

        std::string status_msg = "grasp success";
        if (req.hand == HandControlRequest::HAND_LEFT) {
          if (g_srv.response.angles[0] < req.thre_warn) {
            res.success = false;
            status_msg = "grasp bad";
          }
          if (g_srv.response.angles[0] > req.thre_fail) {
            res.success = false;
            status_msg = "grasp failed";
          }
        } else if (req.hand == HandControlRequest::HAND_RIGHT) {
          if (g_srv.response.angles[1] > -req.thre_warn) {
            res.success = false;
            status_msg = "grasp bad";
          }
          if (g_srv.response.angles[1] < -req.thre_fail) {
            res.success = false;
            status_msg = "grasp failed";
          }
        }

        res.status = status_msg;
      }
      break;
    case HandControlRequest::COMMAND_UNGRASP:
      {
        OpenHand(req.hand);
        res.status = "ungrasp success";
      }
      break;
    case HandControlRequest::COMMAND_GRASP_ANGLE:
      {
        GraspAngle(req.hand, req.larm_angle, req.rarm_angle);
        res.status = "grasp-angle success";
      }
      break;
    case HandControlRequest::COMMAND_GRASP_FAST:
      {
        // grasp till angle, check if grasp is okay, then hold
        auto joints = GraspAngle(req.hand, 0.0, 0.0, 1.2); // grasp takes about 1 sec

        if (req.hand == HandControlRequest::HAND_LEFT) {
          int at = static_cast<int> // should always be found
            (std::find(joints.joint_names.begin(), joints.joint_names.end(),
                       "l_thumb_joint") - joints.joint_names.begin());
          if (fabs(joints.points.positions.at(at)) < 0.05) {
            if (fabs(req.larm_angle) < 0.05) {
              OpenHand(req.hand);
            } else {
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
          int at = static_cast<int> // should always be found
            (std::find(joints.joint_names.begin(), joints.joint_names.end(),
                       "r_thumb_joint") - joints.joint_names.begin());
          if (fabs(joints.points.positions.at(at)) < 0.05) {
            if (fabs(req.rarm_angle) < 0.05) {
              OpenHand(req.hand);
            } else {
              GraspAngle(req.hand, 0.0, req.rarm_angle, 1.2);
            }
            res.success = false;
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

        // call service
        g_client_.call(g_srv);

        std::string status_msg = "grasp success";
        if (req.hand == HandControlRequest::HAND_LEFT) {
          if (g_srv.response.angles[0] > req.thre_fail) {
            res.success = false;
            status_msg = "grasp failed";
          }
        } else if (req.hand == HandControlRequest::HAND_RIGHT) {
          if (g_srv.response.angles[1] < -req.thre_fail) {
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
    aero_startup::GraspControl g_srv;

    if (hand == HandControlRequest::HAND_LEFT) {
      g_srv.request.position = POSITION_Left;
      g_srv.request.script = GraspControlRequest::SCRIPT_UNGRASP;
      executing_flg_left_ = false;
    } else if (hand == HandControlRequest::HAND_RIGHT) {
      g_srv.request.position = POSITION_Right;
      g_srv.request.script = GraspControlRequest::SCRIPT_UNGRASP;
      executing_flg_right_ = false;
    }
    g_srv.request.power = (100 << 8) + 30;
    // call service
    g_client_.call(g_srv);
  }

  aero_startup::AeroSendJoints::Response GraspAngle
  (int hand, float larm_angle, float rarm_angle, float time=0.5)
  {
    aero_startup::AeroSendJoints srv;
    switch(hand) {
    case HandControlRequest::HAND_BOTH:
      srv.request.joint_names = {"l_thumb_joint", "r_thumb_joint"};
      srv.request.points.positions.resize(2);
      srv.request.points.positions[0] = larm_angle * M_PI / 180;
      srv.request.points.positions[1] = rarm_angle * M_PI / 180;
      break;
    case HandControlRequest::HAND_LEFT:
      if (executing_flg_left_) {
        aero_startup::GraspControl g_srv;
        g_srv.request.position = POSITION_Left;
        g_srv.request.script = GraspControlRequest::SCRIPT_CANCEL;
        g_srv.request.power = (100 << 8) + 30;
        // call service
        g_client_.call(g_srv);
        executing_flg_left_ = false;
      }
      srv.request.joint_names = {"l_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = larm_angle * M_PI / 180;
      break;
    case HandControlRequest::HAND_RIGHT:
      if (executing_flg_right_) {
        aero_startup::GraspControl g_srv;
        g_srv.request.position = POSITION_Right;
        g_srv.request.script = GraspControlRequest::SCRIPT_CANCEL;
        g_srv.request.power = (100 << 8) + 30;
        // call service
        g_client_.call(g_srv);
        executing_flg_right_ = 0;
      }
      srv.request.joint_names = {"r_thumb_joint"};
      srv.request.points.positions.resize(1);
      srv.request.points.positions[0] = rarm_angle * M_PI / 180;
      break;
    }
    //
    srv.request.points.time_from_start = ros::Duration(time);
    client_.call(srv);

    return srv.response;
  }

private:
  bool executing_flg_left_;
  bool executing_flg_right_;
  ros::ServiceClient client_;
  ros::ServiceClient g_client_;
  ros::ServiceServer service_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aero_hand_controller");
  ros::NodeHandle nh;
  AeroHandControl ahc(nh);

  ros::spin();

  return 0;
}
