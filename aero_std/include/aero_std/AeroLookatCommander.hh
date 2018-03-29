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
    public: void disableTrackingMode();
    public: bool setTrackingMode(aero::tracking _mode, const aero::Vector3 &_pos);
    public: bool setLookAtTopic(const std::string &_topic);
    public: bool setNeckRPY(double _r, double _p, double _y);
    public: void getNeckRPY(double &_r, double &_p, double &_y);
    protected: void sendNeckOnce_(const aero::Vector3 &_pos);

    protected: ros::CallbackQueue eventqueue_;
    protected: void timerCallback(const ros::TimerEvent& ev);
    protected: ros::CallbackQueue subqueue_;
    protected: void subCallback(const geometry_msgs::Point::ConstPtr &_msg);

    protected: aero::tracking tracking_mode_;
    protected: aero::Vector3 tracking_pos_;
    protected: double roll_max_vel_;
    protected: double pitch_max_vel_;
    protected: double yaw_max_vel_;
    protected: ros::Time prev_cb_tm_;

    protected: boost::mutex callback_mtx_;
    protected: robot_state::RobotStatePtr kinematic_state_;

    protected: ros::Subscriber sub_;
    protected: ros::Timer event_timer_;
    protected: boost::shared_ptr < ros::AsyncSpinner > event_spinner_;
    protected: boost::shared_ptr < ros::AsyncSpinner > sub_spinner_;

    protected: aero::interface::AeroMoveitInterface *ami_;
    };
  }
}

#endif //
