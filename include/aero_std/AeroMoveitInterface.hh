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
    bool solveIK(std::string _move_group, geometry_msgs::Pose _pose);
    void viewTrajectory();
    void setStartStateToCurrentState(std::string _move_group);
    bool move(std::string _move_group);

    moveit::planning_interface::MoveGroup &getMoveGroup(std::string _move_group);

    // change lifter mode
    void switchOnPlane();
    void switchHeightOnly();

    //
    void setNamedTarget(std::string _move_group, std::string _target);

    void moveWaist(double _x, double _z);

    // for grasp
    bool solveIKSequence(aero::GraspRequest &_grasp);
    std::string solveIKOneSequence(aero::arm _arm, geometry_msgs::Pose _pose, aero::ikrange _ik_range, std::vector<double> _av_ini, std::vector<double> &_result);

    bool moveSequence();

    bool openHand(bool _yes, aero::arm _arm);

    bool openHand(bool _yes, aero::arm _arm, float _warn, float _fail);

    bool openHand(float _angle, aero::arm _arm);

    bool openHand(float _angle, aero::arm _arm, float _warn, float _fail);

  private:
    void getRobotStateVariables(std::vector<double> &_av);

    ros::Publisher display_publisher_;
    moveit::planning_interface::MoveGroup::Plan plan_;
    std::string planned_group_;
    bool height_only_;
    std::vector<std::vector<double>> trajectory_;
    std::vector<std::string> trajectory_groups_;

  };
  typedef std::shared_ptr<AeroMoveitInterface> AeroMoveitInterfacePtr;
  }
}
#endif
