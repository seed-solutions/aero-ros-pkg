#ifndef _AERO_LOOKAT_COMMANDER_
#define _AERO_LOOKAT_COMMANDER_

namespace aero
{
  namespace interface
  {
    class AeroMoveitInterface;
  }
}
namespace aero
{
  namespace lookat_commander
  {
    class AeroLookatCommander;
  }
}
#include <aero_std/AeroMoveitInterface.hh>

namespace aero
{
  namespace lookat_commander
  {
    class AeroLookatCommander
    {
    public: typedef boost::shared_ptr<AeroLookatCommander > Ptr;

    public: AeroLookatCommander(ros::NodeHandle &_nh,
                                aero::interface::AeroMoveitInterface *_ami);

    public: bool setTrackingMode(aero::tracking _mode, const aero::Vector3 &_pos);
    public: bool setNeckRPY(double _r, double _p, double _y);
    public: bool setLookAtTopic(const std::string &_topic);
    public: std::tuple<double, double, double> getNeck();

    protected: ros::CallbackQueue eventqueue_;
    protected: void timerCallback(const ros::TimerEvent& ev);

    protected: aero::tracking tracking_mode_;
    protected: aero::Vector3 tracking_pos_;
    protected: boost::mutex callback_mtx_;
    protected: robot_state::RobotStatePtr kinematic_state_;
    protected: boost::shared_ptr<aero::interface::AeroMoveitInterface > ami_;
    protected: ros::Time previous_callback_;
    };
  }
}

#endif //
