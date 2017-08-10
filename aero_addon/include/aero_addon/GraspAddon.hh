#ifndef _AERO_ADDON_GRASP_ADDON_
#define _AERO_ADDON_GRASP_ADDON_

#include "aero_std/parse.h"
#include "aero_std/AeroMoveitInterface.hh"

namespace aero
{
  namespace addon
  {

    class GraspAddon
    {
    public: explicit GraspAddon
    (ros::NodeHandle _nh, aero::interface::AeroMoveitInterfacePtr _robot);

    public: ~GraspAddon();

    public: bool readTumbleParameters(std::string _file);

    // _target must be world coordinates
    public: bool solveBoxTumbleGrasp
    (aero::arm _arm, Eigen::Vector3f _target, float _height, float _r=0.025f);

    public: bool sendBoxTumbleGrasp(aero::arm _arm, int _power=100);

    protected: bool tmbInitiateBoxTumble
    (aero::arm _arm, Eigen::Vector3f _target, float _d, float _omega);

    protected: bool tmbAdjustInitialHandHeight
    (aero::arm _arm, float _z, float _height, float _level_margin);

    protected: bool tmbSearchHandPose(aero::arm _arm, float _d);

    protected: bool tmbReachHandForward(aero::arm _arm, float _x, float _max_reach);

    protected: bool tmbTiltHandBackward
    (float _theta, float _height, float _r, float _press_param);

    protected: ros::NodeHandle nh_;

    protected: aero::interface::AeroMoveitInterfacePtr robot_;

    protected: geometry_msgs::Pose tmb_eef_;

    protected: std::vector<std::map<aero::joint, double> > tmb_av_;

    protected: std::vector<std::pair<double, double> > tmb_lifter_av_;

    protected: std::vector<std::map<aero::joint, double> > tmb_reach_av_;

    protected: bool tmb_loaded_;

    protected: float tmb_press_param_;

    protected: float tmb_angle_param_;

    protected: float tmb_reach_param_;

    protected: float tmb_d_;

    protected: float tmb_omega_;

    protected: float tmb_level_margin_;

    protected: Eigen::Vector3d target_;
    };

    typedef std::shared_ptr<GraspAddon> GraspAddonPtr;

  }
}

#endif
