#ifndef _AERO_MOVEIT_INTERFACE_
#define _AERO_MOVEIT_INTERFACE_

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>

#include <tf/transform_listener.h>

#include <aero_std/IKSettings.hh>
#include <aero_std/GraspRequest.hh>
#include <aero_std/interpolation_type.h>
#include <aero_startup/AeroSendJoints.h>
#include <aero_startup/AeroHandController.h>
#include <aero_startup/AeroTorsoController.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <aero_startup/GetSpot.h>
#include <nav_msgs/GetPlan.h>
namespace aero
{
  typedef std::vector<std::map<aero::joint, double>> trajectory;
  namespace interface
  {
    class AeroMoveitInterface
    {
      /// @brief constructor
      /// @param _nh ros node handler
      /// @param _rd robot_description's name, "_rd", "_rd_ho" and "_rd_op" will be loaded
    public: explicit AeroMoveitInterface(ros::NodeHandle _nh, std::string _rd="robot_description");
    public: ~AeroMoveitInterface();

      // ------------------------------------------------------------
      // set robot model's states
      // ------------------------------------------------------------
    public: void setRobotStateVariables(std::vector<double> &_av);
    public: void setRobotStateVariables(std::map<std::string, double> &_map);
    public: void setRobotStateVariables(std::map<aero::joint, double> &_map);

    public: void setRobotStateToCurrentState();
    public: void setRobotStateToNamedTarget(std::string _move_group, std::string _target);

    public: bool setFromIK(std::string _move_group, geometry_msgs::Pose _pose, std::string _eef_link="");
    public: bool setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, std::string _eef_link="");
    public: bool setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, aero::eef _eef);

    public: bool setLifter(double _x, double _z);
    private: bool lifter_ik_(double _x, double _z, std::vector<double>& _ans_xz);

    public: void setLookAt(double _x, double _y, double _z);
    public: void setLookAt(Eigen::Vector3d _target);
    public: void setLookAt(Eigen::Vector3f _target);
    public: void setLookAt(geometry_msgs::Pose _pose);
    public: void resetLookAt();
    public: void setNeck(double _r,double _p, double _y);
    private: void lookAt_(double _x, double _y, double _z);

    public: void setHand(aero::arm _arm, double _radian);
    private: void setHandsFromJointStates_();

    public: void updateLinkTransforms();

      // ------------------------------------------------------------
      // set modes
      // ------------------------------------------------------------
    public: bool setInterpolation(int _i_type);

    public: void setTrackingMode(bool _yes);// future
    public: void switchOnPlane();
    public: void switchHeightOnly();

      // ------------------------------------------------------------
      // get robot model's states
      // ------------------------------------------------------------
    public: void getRobotStateVariables(std::vector<double> &_av);
    public: void getRobotStateVariables(std::map<std::string, double> &_map);
    public: void getRobotStateVariables(std::map<aero::joint, double> &_map);

    public: void getResetManipPose(std::map<aero::joint, double> &_map);

    public: Eigen::Vector3d getWaistPosition();
    public: void getLifter(std::map<aero::joint, double>& _xz);

    public: double getHand(aero::arm _arm);

    public: Eigen::Vector3d getEEFPosition(aero::arm _arm, aero::eef _eef);
    public: Eigen::Quaterniond getEEFOrientation(aero::arm _arm, aero::eef _eef);

    public: moveit::planning_interface::MoveGroup &getMoveGroup(std::string _move_group);
    public: moveit::planning_interface::MoveGroup &getMoveGroup(aero::arm _arm, aero::ikrange _range);

      // ------------------------------------------------------------
      // send to real robot
      // ------------------------------------------------------------
    public: void sendResetManipPose(int _time_ms=3000);

    public: void sendAngleVector(aero::arm _arm, aero::ikrange _range, int _time_ms); // _av in kinematic_state is used
    public: void sendAngleVector(int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso); // all angles from kinematic_state is published
    public: void sendAngleVector(std::map<aero::joint, double> _av_map, int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso);
    public: void sendAngleVectorAsync(aero::arm _arm, aero::ikrange _range, int _time_ms); // _av in kinematic_state is used
    public: void sendAngleVectorAsync(int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso); // all angles from kinematic_state is published
    public: void sendAngleVectorAsync(std::map<aero::joint, double> _av_map, int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso);

    public: bool sendTrajectory(aero::trajectory _trajectory, std::vector<int> _times, aero::ikrange _move_lifter=aero::ikrange::torso);

    public: bool sendTrajectory(aero::trajectory _trajectory, int _time_ms, aero::ikrange _move_lifter=aero::ikrange::torso);

    public: bool sendTrajectoryAsync(aero::trajectory _trajectory, std::vector<int> _times, aero::ikrange _move_lifter=aero::ikrange::torso);

    public: bool sendTrajectoryAsync(aero::trajectory _trajectory, int _time_ms, aero::ikrange _move_lifter=aero::ikrange::torso);

    private: void sendAngleVectorAsync_(const std::vector<double> _av, const std::vector<std::string> _joint_names, const int _time_ms);
    private: void sendAngleVectorAsync_(std::string _move_group, int _time_ms); // _av in kinematic_state is used

    public: bool sendLifter(double _x, double _z, int _time_ms=5000); // m
    public: bool sendLifter(int _x, int _z, int _time_ms=5000); // mm deprecated
    public: bool sendLifterLocal(double _x, double _z, int _time_ms=5000);
    public: bool sendLifterLocal(int _x, int _z, int _time_ms=5000);

    public: bool sendLifterAsync(double _x, double _z, int _time_ms=5000); // m
    public: bool sendLifterAsync(int _x, int _z, int _time_ms=5000); // mm  deprecated
    public: bool sendLifterLocalAsync(double _x, double _z, int _time_ms=5000);
    public: bool sendLifterLocalAsync(int _x, int _z, int _time_ms=5000);

    public: bool sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times);
    public: bool sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, int _time_ms);
    public: bool sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times);
    public: bool sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, int _time_ms);


    public: bool sendGrasp(aero::arm _arm, int _power=100);
    public: bool openHand(aero::arm _arm);
    public: bool sendHand(aero::arm _arm, double _rad);


    public: bool solveIKSequence(aero::GraspRequest &_grasp);
    public: std::string solveIKOneSequence(aero::arm _arm, geometry_msgs::Pose _pose, aero::ikrange _ik_range, std::vector<double> _av_ini, std::string _eef_link, std::vector<double> &_result);
    public: bool sendSequence(std::vector<int> _msecs={5000, 5000});
    public: bool sendPickIK(aero::GraspRequest &_grasp);
    public: bool sendPlaceIK(aero::GraspRequest &_grasp, double _push_height=0.03);

      // ------------------------------------------------------------
      // move_base functions
      // ------------------------------------------------------------
    public: geometry_msgs::Pose getCurrentPose(std::string _map="/map");
    public: geometry_msgs::Pose getLocationPose(std::string _location);
    public: bool goPos(double _x, double _y, double _rad, int _timeout_ms=20000);
    public: void goPosAsync(double _x, double _y, double _rad);
    public: void moveToAsync(std::string _location);
    public: void moveToAsync(Eigen::Vector3d _point);
    public: void moveToAsync(geometry_msgs::Pose _pose);
    public: bool isMoving();
    public: bool at(std::string _location, double _thre=0.2);
    public: bool at(geometry_msgs::Pose _pose, double _thre=0.2);
    public: void stop();
    public: void go();
    public: float toDestination(std::string _location);
    public: void faceTowardAsync(std::string _location);
    public: void faceTowardAsync(geometry_msgs::Pose _pose);
    public: bool checkMoveTo(geometry_msgs::Pose _pose);
    private: bool goPosTurnOnly_(double _rad, int _timeout_ms=20000);


      // these varables are to use moveit libralies
      // don't care
    private: robot_model_loader::RobotModelLoader robot_model_loader_;
    public: robot_model::RobotModelPtr kinematic_model;
    public: robot_state::RobotStatePtr kinematic_state;
      // lifter height only
    private: robot_model_loader::RobotModelLoader robot_model_loader_ho_;
    public: robot_model::RobotModelPtr kinematic_model_ho;
    public: robot_state::RobotStatePtr kinematic_state_ho;
      // lifter on plane
    private: robot_model_loader::RobotModelLoader robot_model_loader_op_;
    public: robot_model::RobotModelPtr kinematic_model_op;
    public: robot_state::RobotStatePtr kinematic_state_op;

      // MoveGroup
    public: moveit::planning_interface::MoveGroup larm;
    public: moveit::planning_interface::MoveGroup larm_with_torso;
    public: moveit::planning_interface::MoveGroup larm_with_lifter;
    public: moveit::planning_interface::MoveGroup rarm;
    public: moveit::planning_interface::MoveGroup rarm_with_torso;
    public: moveit::planning_interface::MoveGroup rarm_with_lifter;
    public: moveit::planning_interface::MoveGroup lifter;
    public: moveit::planning_interface::MoveGroup upper_body;
    public: moveit::planning_interface::MoveGroup torso;
    public: moveit::planning_interface::MoveGroup head;

      // JointModelGroup
    public: const robot_state::JointModelGroup* jmg_larm;
    public: const robot_state::JointModelGroup* jmg_larm_with_torso;
    public: const robot_state::JointModelGroup* jmg_larm_with_lifter;
    public: const robot_state::JointModelGroup* jmg_larm_with_lifter_ho;
    public: const robot_state::JointModelGroup* jmg_larm_with_lifter_op;
    public: const robot_state::JointModelGroup* jmg_rarm;
    public: const robot_state::JointModelGroup* jmg_rarm_with_torso;
    public: const robot_state::JointModelGroup* jmg_rarm_with_lifter;
    public: const robot_state::JointModelGroup* jmg_rarm_with_lifter_ho;
    public: const robot_state::JointModelGroup* jmg_rarm_with_lifter_op;
    public: const robot_state::JointModelGroup* jmg_lifter;

      // planning scene interface
    public: moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      // moveit functions
    public: bool plan(std::string _move_group);
    public: bool execute();
    public: void viewTrajectory();
    public: void setStartStateToCurrentState(std::string _move_group);
    public: void setNamedTarget(std::string _move_group, std::string _target);
    public: bool move(std::string _move_group);

      // with linux_kinect
    public: void speakAsync(std::string _speech);
    public: void speak(std::string _speech, float _wait_sec);
    public: void beginListen();
    public: void endListen();
    public: std::string listen();


      // callback functions
    private: void JointStateCallback_(const sensor_msgs::JointState::ConstPtr &_msg);
    private: void listenerCallBack_(const std_msgs::String::ConstPtr& _msg);


    private: ros::ServiceClient hand_grasp_client_;
    private: ros::ServiceClient joint_states_client_;
    private: ros::ServiceClient interpolation_client_;
    private: ros::ServiceClient activate_tracking_client_;
    private: ros::Publisher display_publisher_;
    private: ros::Publisher angle_vector_publisher_;
    private: ros::Publisher look_at_publisher_;
    private: ros::Publisher speech_publisher_;
    private: ros::Publisher speech_detection_settings_publisher_;
    private: ros::Publisher cmd_vel_publisher_;
    private: ros::Subscriber joint_states_subscriber_;
    private: ros::Subscriber speech_listener_;
    private: ros::ServiceClient waist_service_;
    private: ros::ServiceClient lifter_ik_service_;
    private: ros::ServiceClient send_angle_service_;
    private: ros::ServiceClient get_spot_;
    private: ros::ServiceClient check_move_to_;
    private: actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_;
    private: moveit::planning_interface::MoveGroup::Plan plan_;
    private: std::string planned_group_;
    private: bool height_only_;
    private: std::vector<std::vector<double>> trajectory_;
    private: std::vector<std::string> trajectory_groups_;
    private: sensor_msgs::JointState joint_states_;
    private: double lifter_thigh_link_;// lifter's upper link
    private: double lifter_foreleg_link_;// lifter's lower link
    private: std::string detected_speech_;
    private: bool tracking_mode_flag_;
    private: aero_startup::AeroSendJoints send_joints_srv_;
    private: tf::TransformListener listener_;
    private: geometry_msgs::Pose pose_using_;

    };
    typedef std::shared_ptr<AeroMoveitInterface> AeroMoveitInterfacePtr;
  }
}
#endif
