#ifndef _AERO_LOOKAT_COMMANDER_
#define _AERO_LOOKAT_COMMANDER_

#include <aero_std/AeroMoveitInterface.hh>

namespace aero
{
  namespace lookat_commander
  {
    class AeroLookatCommander
    {
    public: AeroLookatCommander(aero::interface::AeroMoveitInterface *_ami);

    protected: ros::CallbackQueue eventqueue_;
    protected: void timerCallback(const ros::TimerEvent& ev);

    protected: boost::mutex interface_mtx_;
    protected: robot_state::RobotStatePtr kinematic_state_;
    protected: aero::interface::AeroMoveitInterface::Ptr ami_;
    };
  }
}

#endif //
