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

#include <aero_std/AeroInterface.hh>
#include <aero_std/IKSettings.hh>
#include <aero_std/GraspRequest.hh>


namespace aero
{
  namespace interface
  {
  class AeroMoveitInterface : public AeroInterface
  {
  public:
    //explicit AeroMoveitInterface();

    /// @brief constructor
    /// @param _nh ros node handler
    /// @param _rd robot_description's name, "_rd", "_rd_ho" and "_rd_op" will be loaded
    explicit AeroMoveitInterface(ros::NodeHandle _nh, std::string _rd);
    ~AeroMoveitInterface();
    
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    // lifter height only
    robot_model_loader::RobotModelLoader robot_model_loader_ho;
    robot_model::RobotModelPtr kinematic_model_ho;
    robot_state::RobotStatePtr kinematic_state_ho;
    // lifter on plane
    robot_model_loader::RobotModelLoader robot_model_loader_op;
    robot_model::RobotModelPtr kinematic_model_op;
    robot_state::RobotStatePtr kinematic_state_op;
    
    // MoveGroup
    moveit::planning_interface::MoveGroup larm;
    moveit::planning_interface::MoveGroup larm_with_torso;
    moveit::planning_interface::MoveGroup larm_with_lifter;
    moveit::planning_interface::MoveGroup rarm;
    moveit::planning_interface::MoveGroup rarm_with_torso;
    moveit::planning_interface::MoveGroup rarm_with_lifter;
    moveit::planning_interface::MoveGroup lifter;
    moveit::planning_interface::MoveGroup upper_body;
    moveit::planning_interface::MoveGroup torso;
    moveit::planning_interface::MoveGroup head;
    
    // JointModelGroup
    const robot_state::JointModelGroup* jmg_larm;
    const robot_state::JointModelGroup* jmg_larm_with_torso;
    const robot_state::JointModelGroup* jmg_larm_with_lifter;
    const robot_state::JointModelGroup* jmg_larm_with_lifter_ho;
    const robot_state::JointModelGroup* jmg_larm_with_lifter_op;
    const robot_state::JointModelGroup* jmg_rarm;
    const robot_state::JointModelGroup* jmg_rarm_with_torso;
    const robot_state::JointModelGroup* jmg_rarm_with_lifter;
    const robot_state::JointModelGroup* jmg_rarm_with_lifter_ho;
    const robot_state::JointModelGroup* jmg_rarm_with_lifter_op;
    const robot_state::JointModelGroup* jmg_lifter;
    
    // planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

    bool plan(std::string _move_group);
    bool execute();
    bool solveIK(std::string _move_group, geometry_msgs::Pose _pose, std::string _eef_link="");
    bool solveIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, std::string _eef_link="");
    bool solveIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, aero::eef _eef);
    void viewTrajectory();
    void setStartStateToCurrentState(std::string _move_group);
    bool move(std::string _move_group);

    moveit::planning_interface::MoveGroup &getMoveGroup(std::string _move_group);
    moveit::planning_interface::MoveGroup &getMoveGroup(aero::arm _arm, aero::ikrange _range);

    // change lifter mode
    void switchOnPlane();
    void switchHeightOnly();

    //
    void setNamedTarget(std::string _move_group, std::string _target);
    void resetManipPose();

    bool moveWaist(double _x, double _z, int _time_ms=0); // m
    bool moveWaist(int _x, int _z, int _time_ms=0); // mm
    bool moveWaistLocal(double _x, double _z, int _time_ms=0);
    bool moveWaistLocal(int _x, int _z, int _time_ms=0);

    bool moveWaistAsync(double _x, double _z, int _time_ms=0); // m
    bool moveWaistAsync(int _x, int _z, int _time_ms=0); // mm
    bool moveWaistLocalAsync(double _x, double _z, int _time_ms=0);
    bool moveWaistLocalAsync(int _x, int _z, int _time_ms=0);

    // set waist position of kinametic_state
    void setWaist(double _x, double _z);
    void setWaist(int _x, int _z);

    std::vector<double> getWaistPosition();

    // for grasp
    bool solveIKSequence(aero::GraspRequest &_grasp);
    std::string solveIKOneSequence(aero::arm _arm, geometry_msgs::Pose _pose, aero::ikrange _ik_range, std::vector<double> _av_ini, std::string _eef_link, std::vector<double> &_result);

    bool moveSequence();

    bool openHand(bool _yes, aero::arm _arm);

    bool openHand(bool _yes, aero::arm _arm, float _warn, float _fail);

    bool openHand(float _angle, aero::arm _arm);

    bool openHand(float _angle, aero::arm _arm, float _warn, float _fail);

    void sendAngleVector(aero::arm _arm, aero::ikrange _range, int _time_ms); // _av in kinematic_state is used

    void sendAngleVector(int _time_ms, bool _move_waist=false); // all angles from kinematic_state is published

    void sendAngleVector(std::map<aero::joint, double> _av_map, int _time_ms, bool _move_waist=false);

    void sendAngleVectorAsync(aero::arm _arm, aero::ikrange _range, int _time_ms); // _av in kinematic_state is used

    void sendAngleVectorAsync(int _time_ms, bool _move_waist=false); // all angles from kinematic_state is published

    void sendAngleVectorAsync(std::map<aero::joint, double> _av_map, int _time_ms, bool _move_waist=false);

    void setRobotStateVariables(std::vector<double> &_av);
    void setRobotStateVariables(std::map<std::string, double> &_map);
    void setRobotStateVariables(std::map<aero::joint, double> &_map);

    void getRobotStateVariables(std::vector<double> &_av);
    void getRobotStateVariables(std::map<std::string, double> &_map);
    void getRobotStateVariables(std::map<aero::joint, double> &_map);

    void setRobotStateToCurrentState();

    void setRobotStateToNamedTarget(std::string _move_group, std::string _target);

    void setHand(aero::arm _arm, int _angle);// insert angle[deg] which is openhand's command degree

    void setHand(aero::arm _arm, double _radian);// insert actual joint angle[rad] from l,r_thumb_joint

    Eigen::Vector3d getThumbPosition(aero::arm _arm);

    Eigen::Vector3d getIndexPosition(aero::arm _arm);

    void updateLinkTransforms();

  private:
    void sendAngleVectorAsync_(const std::vector<double> _av, const std::vector<std::string> _joint_names, const int _time_ms);
    void sendAngleVectorAsync_(std::string _move_group, int _time_ms); // _av in kinematic_state is used

    void setHandsFromJointStates_();

    void JointStateCallback(const sensor_msgs::JointState::ConstPtr &_msg);
    ros::ServiceClient hand_grasp_client_;
    ros::Publisher display_publisher_;
    ros::Publisher angle_vector_publisher_;
    ros::Subscriber joint_states_subscriber_;
    ros::ServiceClient waist_service_;
    moveit::planning_interface::MoveGroup::Plan plan_;
    std::string planned_group_;
    bool height_only_;
    std::vector<std::vector<double>> trajectory_;
    std::vector<std::string> trajectory_groups_;
    sensor_msgs::JointState joint_states_;
  };
  typedef std::shared_ptr<AeroMoveitInterface> AeroMoveitInterfacePtr;
  }
}
#endif
