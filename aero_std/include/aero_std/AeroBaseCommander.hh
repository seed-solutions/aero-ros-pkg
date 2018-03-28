#ifndef _AERO_BASE_COMMANDER_
#define _AERO_BASE_COMMANDER_

/// TODO: new code(move_base)
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>

#include <aero_std/GetSpot.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <aero_std/IKSettings.hh>

namespace aero
{
  namespace base_commander
  {
    class AeroBaseCommander
    {
    public: AeroBaseCommander(ros::NodeHandle &_nh);

      // ------------------------------------------------------------
      // move_base functions
      // ------------------------------------------------------------
      /// @brief get current robot's pose in world
      /// @param[in] _map map name
      /// @return current pose from _map to base_link
    public: bool getCurrentCoords(aero::Transform &_coords, const std::string &_origin_frame="/map");
      /// @brief get named location's pose
      /// @param[in] _location see spot.yaml
      /// @return pose from map to location
    public: bool getLocationCoords(aero::Transform &_coords, const std::string &_location);
      /// @brief move wheel to relative position
      /// @param[in] _x relative x in meters
      /// @param[in] _y relative y in meters
      /// @param[in] _timeout_ms timeout
      /// @param[in] _async asynchronous(return immediately) if true
      /// @return if lifter can't reach the desired position before timeout, return false
    public: bool goPos(double _x, double _y, double _rad, int _timeout_ms=20000, bool _async=false);
      /// @brief move wheel to named location
      /// @attention this function returns immediately after called. you can get the information by calling isMoving
      /// @param[in] _location see spot.yaml
      /// @param[in] _async asynchronous(return immediately) if true
    public: void moveTo(const std::string &_location, bool _async=false);
      /// @brief move wheel to desired position
      /// @attention this function returns immediately after called. you can get the information by calling isMoving
      /// @param[in] _point desired position in map coordinate
      /// @param[in] _async asynchronous(return immediately) if true
    public: void moveTo(const aero::Transform &_coords, bool _async=false);
      /// @brief check asynchronous wheel function is on process or not
      /// @return when wheel is moving, return true
    public: bool isMoving();
      /// @brief check robot is at location or not
      /// @param[in] _location see spot.yaml
      /// @param[in] _thre threshold
      /// @return if base_link is inside thresold in map coordinate, return true
    public: bool isAt(const std::string &_location, double _thre=0.2);
      /// @brief check robot is at _pose or not
      /// @param[in] _pose desired pose
      /// @param[in] _thre threshold
      /// @return if base_link is inside thresold in map coordinate, return true
    public: bool isAt(const aero::Transform &_coords, double _thre=0.2);

      /// @brief stop asynchronous wheel functions
    public: void stop();
      /// @brief re-execure stopped functions
    public: void go();
      /// @brief get the distance to location
      /// @param[in] _location see spot.yaml
      /// @return distance from base_link to location in meters
    public: double toDestination(const std::string &_location);
      /// @brief turn the robot toward location
      /// @param[in] _location see spot.yaml
    public: void faceToward(const std::string &_location);
      /// @brief turn the robot toward pose
      /// @param[in] target pose in map coordinate
    public: void faceToward(const aero::Transform &_coords);

      /// @brief check global cost map wether the robot can make plan to go to the position or not
      /// @param[in] _pose desired pose in map coordinate
      /// @return if wheel moving plan to the pose can be made, return true
    public: bool checkMoveTo(const aero::Transform &_coords);
      /// @brief protected function. due to move_base_pkg's bug, we use this
      // protected: bool goPosTurnOnly_(double _rad, int _timeout_ms=20000);

    public: virtual aero::Vector3 volatileTransformToBase(const aero::Vector3 &_pos);
    public: virtual aero::Vector3 volatileTransformToBase(double _x, double _y, double _z) {
      ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " : this method is deprecated");
      aero::Vector3 pos(_x, _y, _z);
      return volatileTransformToBase(pos);
    }

    protected: tf::TransformListener listener_;

    protected: std::string robot_base_frame;

    protected: ros::Publisher cmd_vel_publisher_;
    protected: ros::ServiceClient get_spot_;
    protected: ros::ServiceClient check_move_to_;
    protected: actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *base_ac_;

    protected: aero::Transform destination_coords_;
    };
  }
}

#endif //
