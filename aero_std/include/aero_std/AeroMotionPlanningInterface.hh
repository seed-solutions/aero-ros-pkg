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

    public: void processAttachedCollisionMesh(const std::string _link_name, const std::string _parent, const aero::Transform _pose, const std::string _resource, const std::vector<std::string> _touch_links);

    /// @brief create goal state for request (note, model will be updated)
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _group_name name of group to do manipulation
    /// @param[in] _goal goal pose of goal state
    /// @param[in] _eef_link name of link referred to for goal state
    /// @param[in] _tolerance1 pose tolerance
    /// @param[in] _tolerance2 angle tolerance
    /// @return constraints to push in request
    public: moveit_msgs::Constraints constructGoalConstraintsFromIK
    (aero::interface::AeroMoveitInterface::Ptr _robot, const std::string _group_name, const aero::Transform _goal, const std::string _eef_link, const double _tolerance1=0.03, const double _tolerance2=0.03);

    /// @brief solve the motion plan
    /// @param[in] _req motion plan settings
    /// @param[in] _res solved trajectory and status
    /// @return whether successful or not
    public: bool solve
    (planning_interface::MotionPlanRequest& _req, planning_interface::MotionPlanResponse& _res);

    public: bool solveEEFCollisionEnabled
    (planning_interface::MotionPlanRequest& _req, planning_interface::MotionPlanResponse& _res, int _times);

    /// @brief create end effector pose constraint
    /// @param[in] _eef_link end effector to constrain
    /// @param[in] _parent quaternion coordinate
    /// @param[in] _q constrain quaternion value
    /// @return constraint instance
    public: moveit_msgs::Constraints constructPathConstraintsFromQuaternion
    (const std::string _eef_link, const std::string _parent, const aero::Quaternion _q);

    /// @brief display trajectory to rviz
    /// @param[in] _res MotionPlanResponse with solved trajectory
    public: void displayTrajectory(const planning_interface::MotionPlanResponse _res);

    public: boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

    public: planning_scene::PlanningScenePtr planning_scene;

    public: planning_interface::PlannerManagerPtr planner_manager;

    private: ros::Publisher display_publisher_;

    private: ros::Publisher contact_publisher_;

    private: moveit::core::RobotStatePtr rs_tmp_;
    };
  }
}

#endif
