#ifndef _MOVEIT_GRASP_REQUEST_H_
#define _MOVEIT_GRASP_REQUEST_H_

#include "aero_std/IKSettings.hh"

namespace aero
{
  struct GraspRequest
  {

  public: GraspRequest() :
    arm(aero::arm::rarm),
    eef(aero::eef::hand),
    mid_pose(aero::Transform::Identity()),
    mid_ik_range(aero::ikrange::torso),
    end_pose(aero::Transform::Identity()),
    end_ik_range(aero::ikrange::torso) {
  }

  public: aero::arm arm;
  public: aero::eef eef;

  public: aero::Transform mid_pose;
  public: aero::ikrange mid_ik_range; // arm, torso, lifter

  public: aero::Transform end_pose;
  public: aero::ikrange end_ik_range;
  };

}

static std::ostream& operator<<(std::ostream& os, const aero::GraspRequest &_g_req)
{
  std::string eef_name = eefLink(_g_req.arm, _g_req.eef);
  std::string mid_range = moveGroup(_g_req.arm, _g_req.mid_ik_range);
  std::string end_range = moveGroup(_g_req.arm, _g_req.end_ik_range);

  os << "grasp: eef = " << eef_name;
  os << ", mid_range = " << mid_range;
  os << ", end_range = " << end_range;
  os << ", mid: " <<  _g_req.mid_pose;
  os << ", end: " <<  _g_req.end_pose;

  return os;
}
#endif
