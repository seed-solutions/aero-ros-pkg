#ifndef _AERO_MOTION_PLANNING_INTERFACE_
#define _AERO_MOTION_PLANNING_INTERFACE_

#include <pluginlib/class_loader.h>
#include <aero_std/AeroMoveitInterface.hh>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

#include <boost/scoped_ptr.hpp>

#include <visualization_msgs/Marker.h>

namespace aero
{
  namespace interface
  {
    class AeroMotionPlanningInterface
    {
    public: typedef std::shared_ptr<AeroMotionPlanningInterface> Ptr;

    /// @brief constructor
    /// @param[in] _nh ros node handler
    /// @param[in] _rm robot model
    /// @param[in] _plugin planner plugin to use (only OMPL for now)
    public: explicit AeroMotionPlanningInterface
    (ros::NodeHandle &_nh, const robot_model::RobotModelConstPtr _rm, const std::string _plugin="ompl_interface/OMPLPlanner");
    public: ~AeroMotionPlanningInterface();

    /// @brief set start state of plan (note, links in model will be updated)
    /// @param[in] robot interface with kinematic model
    public: void setCurrentState(aero::interface::AeroMoveitInterface::Ptr _robot);

    /// @brief transform aero::Transform to transform in move_group eef (aero::eef::hand)
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _group_name name of move_group
    /// @param[in] _arm arm of transform
    /// @param[in] _eef end effector link of transform
    /// @param[in] _mat aero::Transform to transform
    /// @return transformed aero::Transform
    public: aero::Transform getTransformInMoveGroupEEF
    (aero::interface::AeroMoveitInterface::Ptr _robot, const std::string _group_name, const aero::arm _arm, const aero::eef _eef, const aero::Transform _mat);

    /// @brief add box to scene for collision
    /// @param[in] _id name the box
    /// @param[in] _parent where the box should be attached to e.g. map, base_link
    /// @param[in] _pose position and orientation of box from parent link
    /// @param[in] _scale size of box in meters
    public: void processCollisionBox(const std::string _id, const std::string _parent, const aero::Transform _pose, const aero::Vector3 _scale);

    /// @brief add mesh to scene for collision
    /// @param[in] _id name the box
    /// @param[in] _parent where the mesh should be attached to e.g. map, base_link
    /// @param[in] _pose position and orientation of mesh from parent link
    /// @param[in] _resource mesh file name
    public: void processCollisionMesh(const std::string _id, const std::string _parent, const aero::Transform _pose, const std::string _resource);

    // public: void processAttachedCollisionMesh(const std::string _link_name, const std::string _parent, const aero::Transform _pose, const std::string _resource, const std::vector<std::string> _touch_links);

    /// @brief create goal state for request from current robot joint state
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _group_name name of move_group
    /// @param[in] _tolerance1 pose tolerance
    /// @param[in] _tolerance2 angle tolerance
    /// @return constraints to push in request      
    public: moveit_msgs::Constraints constructGoalConstraints
    (aero::interface::AeroMoveitInterface::Ptr _robot, std::string _group_name, const double _tolerance1=0.03, const double _tolerance2=0.03);

    /// @brief create goal state for request from pose
    /// @param[in] _origin name of group to do manipulation
    /// @param[in] _goal goal pose of goal state
    /// @param[in] _arm arm to solve (note, goal state always refers to aero::eef::hand)
    /// @param[in] _tolerance1 pose tolerance
    /// @param[in] _tolerance2 angle tolerance
    /// @return constraints to push in request
    public: moveit_msgs::Constraints constructGoalConstraints
    (const std::string _origin, const aero::Transform _goal, const aero::arm _arm, const double _tolerance1=0.03, const double _tolerance2=0.03);

    /// @brief solve the motion plan
    /// @param[in] _req motion plan settings
    /// @param[in] _res solved trajectory and status
    /// @return whether successful or not
    public: bool plan
    (planning_interface::MotionPlanRequest& _req, planning_interface::MotionPlanResponse& _res);

    /// @brief send the motion plan
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _res solved trajectory and status
    /// @param[in] _duration time to finish whole trajectory in milliseconds
    /// @param[in] _ikrange this should match the range used for solving motion plan
    /// @param[in] _async false to wait interpolation
    /// @return succeed or not
    public: bool execute(aero::interface::AeroMoveitInterface::Ptr _robot, const planning_interface::MotionPlanResponse _res, const int _duration, const aero::ikrange _ikrange, const bool _async=true);

    /// @brief check collision for fast-computed approximate paths
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _res solved trajectory and status
    /// @return whether there was collision or not (visual /moveit/collision/contacts)
    public: bool checkCollision(const aero::interface::AeroMoveitInterface::Ptr _robot, const planning_interface::MotionPlanResponse _res);

    /// @brief create end effector pose constraint
    /// @param[in] _arm arm to constrain (note, constraint always refers to aero::eef::hand)
    /// @param[in] _parent quaternion coordinate
    /// @param[in] _q constrain quaternion value
    /// @return constraint instance
    public: moveit_msgs::Constraints constructPathConstraints
    (const aero::arm _arm, const std::string _parent, const aero::Quaternion _q);

    /// @brief publish objects added to scene
    public: void displayScene();

    /// @brief display trajectory to rviz
    /// @param[in] _res MotionPlanResponse with solved trajectory
    public: void displayTrajectory(const planning_interface::MotionPlanResponse _res);

    public: boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

    public: planning_scene::PlanningScenePtr planning_scene;

    public: planning_interface::PlannerManagerPtr planner_manager;

    public: moveit_msgs::PlanningScene scene_msg;

    private: ros::Publisher display_publisher_;

    private: ros::Publisher scene_publisher_;

    private: ros::Publisher contact_publisher_;
    };
  }
}

#endif
