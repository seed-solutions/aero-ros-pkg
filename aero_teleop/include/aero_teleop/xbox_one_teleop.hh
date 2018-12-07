// for PS3 controller, LT, RT axis should be L2, R2 buttons
// grasp-angle is only for xbox one and PS3 should do ungrasp with L2, R2 without A press

#ifndef AERO_XBOX_ONE_TELEOP_
#define AERO_XBOX_ONE_TELEOP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <aero_std/AeroMoveitInterface.hh>
#include <aero_startup/GraspControl.h>
#include "aero_extra_controllers/AeroHandController.hh"

#include <map>
#include <string>

namespace aero {
  
  namespace interface {
    class hackedAeroMoveitInterface : public aero::interface::AeroMoveitInterface {
    public:
      typedef std::shared_ptr<hackedAeroMoveitInterface> Ptr;
      hackedAeroMoveitInterface(ros::NodeHandle _nh);
      // add any hacking functions if needed
      void hackedSendHand(const aero::arm _arm, const double _rad, const double _t);
      robot_interface::TrajectoryClient::Ptr lhand;
      robot_interface::TrajectoryClient::Ptr rhand;
    };
  }
  
  namespace teleop {

    static const std::string X = "x";
    static const std::string Y = "y";
    static const std::string Z = "z";
    static const std::string ROLL = "roll";
    static const std::string PITCH = "pitch";
    static const std::string YAW = "yaw";
    static const std::string LEFT = "L";
    static const std::string RIGHT = "R";
    static const std::string FLAG = "enable";
    static const std::string RIGHT_FLAG = "enable_right";
    static const std::string LEFT_FLAG = "enable_left";

    static const int BASIC_MODE = 0;
    static const int JOINT_MODE = 1;
    
    class xbox_one_teleop {
    public:
      typedef std::shared_ptr<xbox_one_teleop> Ptr;

      xbox_one_teleop(ros::NodeHandle &nh_param);
      ~xbox_one_teleop();

      void loop();
      
    protected:
      virtual void joyCallback(const sensor_msgs::Joy::ConstPtr &joy_msg);
      void basicMode(const sensor_msgs::Joy::ConstPtr &joy_msg);
      void jointMode(const sensor_msgs::Joy::ConstPtr &joy_msg);
      
      int mode;
      bool during_reset;
      ros::Time reset_start_time;
      bool in_grasp_mode;

      // common
      int reset_pose_button;
      int switch_basic_mode;
      int switch_joint_mode;

      // twist
      int enable_button;
      int enable_turbo_button;
      std::map<std::string, int> axis_linear_map;
      std::map<std::string, double> scale_linear_map;
      std::map<std::string, double> scale_linear_turbo_map;
      std::map<std::string, int> axis_angular_map;
      std::map<std::string, double> scale_angular_map;
      std::map<std::string, double> scale_angular_turbo_map;
      bool sent_disable_msg;

      // other basics
      int external_command_button;
      bool enable_lifter;
      int enable_grasp_angle_button;
      int enable_lifter_button;
      double lifter_dx, lifter_dz;
      std::map<std::string, int> lifter_axis_map;
      std::map<std::string, double> scale_lifter_map;
      std::map<std::string, int> grasp_angle_axis_map;
      std::map<std::string, double> range_grasp_angle_map;
      std::map<std::string, double> min_grasp_angle_map;
      int grasp_L_button;
      int grasp_R_button;

      // joint mode
      bool enable_torso;
      double torso_dy, torso_dp;
      std::map<std::string, int> torso_map;
      std::map<std::string, double> scale_torso_map;
      bool enable_left_shoulder;
      bool enable_right_shoulder;
      double left_shoulder_dr, left_shoulder_dp, left_shoulder_dy;
      double right_shoulder_dr, right_shoulder_dp, right_shoulder_dy;
      std::map<std::string, int> shoulder_map;
      std::map<std::string, double> scale_shoulder_map;
      bool enable_left_elbow;
      bool enable_right_elbow;
      double left_elbow_dp, left_elbow_dy;
      double right_elbow_dp, right_elbow_dy;
      std::map<std::string, int> elbow_map;
      std::map<std::string, double> scale_elbow_map;
      bool enable_left_wrist;
      bool enable_right_wrist;
      double left_wrist_dp, left_wrist_dy;
      double right_wrist_dp, right_wrist_dy;
      std::map<std::string, int> wrist_map;
      std::map<std::string, double> scale_wrist_map;
      bool enable_head;
      double head_dp, head_dy;
      std::map<std::string, int> head_map;
      std::map<std::string, double> scale_head_map;
      
      ros::Publisher cmd_vel_pub;
      interface::hackedAeroMoveitInterface::Ptr robot;
      std::map<aero::joint, double> min_bounds, max_bounds;
      ros::Subscriber joy_sub;
    };

  }
}

#endif
