#ifndef _AERO_MOVE_GROUP_INTERFACE_
#define _AERO_MOVE_GROUP_INTERFACE_

#ifdef KINETIC
#include <moveit/move_group_interface/move_group_interface.h>
#else
#include <moveit/move_group_interface/move_group.h>
#endif
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

namespace aero
{
#ifdef KINETIC
  typedef moveit::planning_interface::MoveGroupInterface MoveGroupClass;
#else
  typedef moveit::planning_interface::MoveGroup          MoveGroupClass;
#endif
  namespace interface
  {
    class AeroMoveGroupInterface
    {
    public: typedef std::shared_ptr<AeroMoveGroupInterface> Ptr;

    public: AeroMoveGroupInterface(const std::string &_group_name,
                                   const std::string &_rd="robot_description") {

      move_group_nh.reset (new ros::NodeHandle());
      move_group_nh->setCallbackQueue(&move_group_queue);
      spinner.reset(new ros::AsyncSpinner (0, &move_group_queue));
      spinner->start();

      MoveGroupClass::Options
        opt(_group_name, _rd, *move_group_nh);
      move_group.reset(new MoveGroupClass(opt));

      scene.reset(new moveit::planning_interface::PlanningSceneInterface());
    }

    public: ~AeroMoveGroupInterface() {
      spinner->stop();
    }

      // public:
    public: std::shared_ptr<aero::MoveGroupClass > move_group;
    public: std::shared_ptr<moveit::planning_interface::PlanningSceneInterface > scene;

      // private:
    private: std::shared_ptr<ros::NodeHandle > move_group_nh;
    private: std::shared_ptr<ros::AsyncSpinner > spinner;
    private: ros::CallbackQueue move_group_queue;
    };
  }
}

#endif
