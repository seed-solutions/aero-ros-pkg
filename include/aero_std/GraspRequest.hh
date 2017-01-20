#ifndef _MOVEIT_GRASP_REQUEST_H_
#define _MOVEIT_GRASP_REQUEST_H_

namespace aero
{
  struct GraspRequest
  {

  public: GraspRequest() :
    arm("right"),
    mid_pose(geometry_msgs::Pose()),
    mid_ik_range("arm"),
    end_pose(geometry_msgs::Pose()),
    end_ik_range("arm") {}


  public: std::string arm;

  public: geometry_msgs::Pose mid_pose;

  public: std::string mid_ik_range; // arm, torso, lifter

  public: geometry_msgs::Pose end_pose;

  public: std::string end_ik_range;
  };
}

#endif
