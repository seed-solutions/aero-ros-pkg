#include <aero_std/AeroLookatCommander.hh>

aero::lookat_commander::AeroLookatCommander::AeroLookatCommander(aero::interface::AeroMoveitInterface *_ami)
{
  ami_.reset(_ami);
  kinematic_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(ami_->kinematic_model));

  ros::TimerOptions(ros::Duration(0.1),
                    boost::bind(&aero::lookat_commander::AeroLookatCommander::timerCallback,
                                this, _1),
                    &eventqueue_);
}

void aero::lookat_commander::AeroLookatCommander::timerCallback(const ros::TimerEvent& ev)
{

}
