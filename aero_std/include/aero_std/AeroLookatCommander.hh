#ifndef _AERO_LOOKAT_COMMANDER_
#define _AERO_LOOKAT_COMMANDER_

#include <aero_std/AeroMoveitInterface.hh>

namespace aero
{
  namespace lookat_commander
  {
    class AeroLookatCommander
    {
    public: AeroLookatCommander(ros::NodeHandle &_nh,
                                aero::interface::AeroMoveitInterface *_ami);

    public: bool setTrackingMode(aero::tracking _mode, const aero::Vector3 &_pos);
    public: bool setNeckRPY(double _r, double _p, double _y);
    public: bool setLookAtTopic(const std::string &_topic);

    protected: ros::CallbackQueue eventqueue_;
    protected: void timerCallback(const ros::TimerEvent& ev);

    protected: aero::tracking tracking_mode_;
    protected: aero::Vector3 tracking_pos_;
    protected: boost::mutex callback_mtx_;
    protected: robot_state::RobotStatePtr kinematic_state_;
    protected: aero::interface::AeroMoveitInterface::Ptr ami_;
    protected: ros::Time previous_callback_;
    };
  }
}

#endif //
