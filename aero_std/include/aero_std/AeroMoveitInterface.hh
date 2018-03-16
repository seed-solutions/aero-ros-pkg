#ifndef _AERO_MOVEIT_INTERFACE_
#define _AERO_MOVEIT_INTERFACE_

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

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>

#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <aero_std/IKSettings.hh>
#include <aero_std/GraspRequest.hh>
#include <aero_std/interpolation_type.h>

// msgs
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
// srvs
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <aero_std/GetSpot.h>
#include <nav_msgs/GetPlan.h>
#include <aero_startup/AeroSendJoints.h>
#include <aero_startup/AeroTorsoController.h>
#include <aero_startup/HandControl.h>

#include <mutex>

namespace aero
{
#ifdef KINETIC
  typedef moveit::planning_interface::MoveGroupInterface AeroMoveGroup;
#else
  typedef moveit::planning_interface::MoveGroup AeroMoveGroup;
#endif

  typedef std::vector<aero::joint_angle_map> trajectory;
  namespace interface
  {
    class AeroMoveitInterface
    {
      /// @brief constructor
      /// @param[in] _nh ros node handler
      /// @param[in] _rd robot_description's name, "${_rd}", "${_rd}_ho" and "${_rd}_op" will be loaded
    public: explicit AeroMoveitInterface(ros::NodeHandle _nh, std::string _rd="robot_description");
    public: ~AeroMoveitInterface();

      // ------------------------------------------------------------
      // set robot model's states
      // ------------------------------------------------------------

      /// @brief set robot model's angles
      /// @param[in] _av angle vector
    public: void setRobotStateVariables(std::vector<double> &_av);
      /// @brief set robot model's angles
      /// param[in] _map map<joint name, radian>
    public: void setRobotStateVariables(std::map<std::string, double> &_map);
      /// @brief set robot model's angles, recommend using this!
      /// @param[in] _map map<joint name, radian>
    public: void setRobotStateVariables(aero::joint_angle_map &_map);

      /// @brief set current real robot's angles to robot model's angles
    public: void setRobotStateToCurrentState();
      /// @brief set named angles such as "reset-pose" to robot model's angles
      /// @param[in] _move_group named target is declared with move group
      /// @param[in] _target target name, target list is in .srdf file in aero_moveit_config
    public: void setRobotStateToNamedTarget(std::string _move_group, std::string _target);

      /// @brief solve IK and set result to robot model's angles (this method is deprecated)
      /// @param[in] _move_group IK is solved in this move group
      /// @param[in] _pose IK target pose
      /// @param[in] _eef_link end effector link name, this link gets closer to _pose
      /// @param[in] _attempts the number of times IK is attempted
      /// @return true is solved, false is unsolvable
    public: bool setFromIK(std::string _move_group, geometry_msgs::Pose _pose, std::string _eef_link="", int _attempts=10);
      /// @brief solve IK and set result to robot model's angles (this method is deprecated)
      /// @param[in] _arm aero::arm::rarm or aero::arm::larm
      /// @param[in] _range aero::ikrange::(arm|torso|lifter) is to describe joints used in IK
      /// @param[in] _pose IK target pose
      /// @param[in] _eef_link end effector link name, this link gets closer to _pose
      /// @param[in] _attempts the number of times IK is attempted
      /// @return true is solved, false is unsolvable
    public: bool setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, std::string _eef_link="", int _attempts=10);
      /// @brief solve IK and set result to robot model's angles (this method is deprecated)
      /// @param[in] _arm aero::arm::rarm or aero::arm::larm
      /// @param[in] _range aero::ikrange::(arm|torso|lifter) is to describe joints used in IK
      /// @param[in] _pose IK target pose
      /// @param[in] _eef_link this link gets closer to _pose. you can use aero::eef::(hand|grasp|pick)
      /// @param[in] _attempts the number of times IK is attempted
      /// @return true is solved, false is unsolvable
    public: bool setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, aero::eef _eef, int _attempts=10);

      /// @brief solve IK and set result to robot model's angles (this method is deprecated)
      /// @param[in] _move_group IK is solved in this move group
      /// @param[in] _pos IK target position
      /// @param[in] _qua IK target quaternion
      /// @param[in] _eef_link end effector link name, this link gets closer to _pose
      /// @param[in] _attempts the number of times IK is attempted
      /// @return true is solved, false is unsolvable
    public: bool setFromIK(std::string _move_group, Vector3 _pos, Quaternion _qua, std::string _eef_link="", int _attempts=10);
      /// @brief solve IK and set result to robot model's angles (this method is deprecated)
      /// @param[in] _arm aero::arm::rarm or aero::arm::larm
      /// @param[in] _range aero::ikrange::(arm|torso|lifter) is to describe joints used in IK
      /// @param[in] _pos IK target position
      /// @param[in] _qua IK target quaternion
      /// @param[in] _eef_link end effector link name, this link gets closer to _pose
      /// @param[in] _attempts the number of times IK is attempted
      /// @return true is solved, false is unsolvable
    public: bool setFromIK(aero::arm _arm, aero::ikrange _range, Vector3 _pos, Quaternion _qua, std::string _eef_link="", int _attempts=10);
      /// @brief solve IK and set result to robot model's angles (this method is deprecated)
      /// @param[in] _arm aero::arm::rarm or aero::arm::larm
      /// @param[in] _range aero::ikrange::(arm|torso|lifter) is to describe joints used in IK
      /// @param[in] _pos IK target position
      /// @param[in] _qua IK target quaternion
      /// @param[in] _eef_link this link gets closer to _pose. you can use aero::eef::(hand|grasp|pick)
      /// @param[in] _attempts the number of times IK is attempted
      /// @return true is solved, false is unsolvable
    public: bool setFromIK(aero::arm _arm, aero::ikrange _range, Vector3 _pos, Quaternion _qua, aero::eef _eef, int _attempts=10);

      /// @brief solve IK and set result to robot model's angles
      /// @param[in] _move_group IK is solved in this move group
      /// @param[in] _pos IK target position
      /// @param[in] _qua IK target quaternion
      /// @param[in] _eef_link end effector link name, this link gets closer to _pose
      /// @param[in] _attempts the number of times IK is attempted
      /// @return true is solved, false is unsolvable
    public: bool setFromIK(std::string _move_group, const Transform &_pose, std::string _eef_link="", int _attempts=10);
      /// @brief solve IK and set result to robot model's angles
      /// @param[in] _arm aero::arm::rarm or aero::arm::larm
      /// @param[in] _range aero::ikrange::(arm|torso|lifter) is to describe joints used in IK
      /// @param[in] _pos IK target position
      /// @param[in] _qua IK target quaternion
      /// @param[in] _eef_link this link gets closer to _pose. you can use aero::eef::(hand|grasp|pick)
      /// @param[in] _attempts the number of times IK is attempted
      /// @return true is solved, false is unsolvable
    public: bool setFromIK(aero::arm _arm, aero::ikrange _range, const Transform &_pose, aero::eef _eef=aero::eef::none, int _attempts=10);

      /// @brief set robot model's lifter position
      /// @param[in] _x x meters from top of lifter
      /// @param[in] _z z meters from top of lifter
      /// @param[in] _check_lifter_ik deprecated argument.
      /// @return if lifter position is outside of limit. return false and setting fails
    public: bool setLifter(double _x, double _z, bool _check_lifter_ik=true);
      /// @brief check lifter is solvable or not
      /// @param[in] _x target x
      /// @param[in] _z target z
      /// @param[out] _ans_xz ankle and hip joint value
      /// @return bool solvable or not
    protected: bool lifter_ik_(double _x, double _z, std::vector<double>& _ans_xz);


      /// @brief robot model's neck looks at target, the angle values are sent to real robot when sendAngleVector is called
      /// @param[in] _x target x in base_link coordinate
      /// @param[in] _y target y in base_link coordinate
      /// @param[in] _z target z in base_link coordinate
      /// @param[in] _map_coordinate True if map coordinate. Only valid in tracking mode.
      /// @param[in] _tracking True for tracking (setTrackingMode to true is not sufficient, see setTrackingMode for why).
    public: void setLookAt(double _x, double _y, double _z, bool _map_coordinate=false, bool _tracking=false);
      /// @brief robot model's neck looks at target, the angle values are sent to real robot when sendAngleVector is called
      /// @param[in] _target target pose in base_link coordinate
      /// @param[in] _map_coordinate True if map coordinate. Only valid in tracking mode.
      /// @param[in] _tracking True for tracking (setTrackingMode to true is not sufficient, see setTrackingMode for why).
    public: void setLookAt(Vector3 _target, bool _map_coordinate=false, bool _tracking=false);
      /// @brief robot model's neck looks at target, the angle values are sent to real robot when sendAngleVector is called
      /// @param[in] _target target pose in base_link coordinate
      /// @param[in] _map_coordinate True if map coordinate. Only valid in tracking mode.
      /// @param[in] _tracking True for tracking (setTrackingMode to true is not sufficient, see setTrackingMode for why).
    public: void setLookAt(Eigen::Vector3f _target, bool _map_coordinate=false, bool _tracking=false);
      /// @brief robot model's neck looks at target, the angle values are sent to real robot when sendAngleVector is called
      /// @param[in] _pose target pose in base_link coordinate
      /// @param[in] _map_coordinate True if map coordinate. Only valid in tracking mode.
      /// @param[in] _tracking True for tracking (setTrackingMode to true is not sufficient, see setTrackingMode for why).
    public: void setLookAt(geometry_msgs::Pose _pose, bool _map_coordinate=false, bool _tracking=false);
      /// @brief set zero to robot model's neck angles, the angle values are sent to real robot when sendAngleVector is called
    public: void resetLookAt();
      /// @brief set directly values to robot model's neck angles, the angle values are sent to real robot when sendAngleVector is called
      /// @param[in] _r roll, if it's over limit, it beocomes within limit
      /// @param[in] _p pitch, if it's over limit, it beocomes within limit
      /// @param[in] _y yaw, if it's over limit, it beocomes within limit
    public: void setNeck(double _r,double _p, double _y, bool _to_node=false);
      /// @brief function to calculate look at
      /// @param[in] _obj target x,y,z in base_link coordinate
    public: std::tuple<double, double, double> solveLookAt(Eigen::Vector3d _obj);
      /// @brief send neck values
      /// @param[in] _time_ms execution time
    public: void sendNeckAsync(int _time_ms=1000);
      /// @brief Set lookAt with external lookAt manager.
      /// @param[in] _topic Name of topic lookAt manager should subscribe.
    public: void setLookAtTopic(std::string _topic, bool _record_topic=false);
      /// @brief Return last set topic for lookAt manager.
      /// @return Last set topic name.
    public: std::string getLookAtTopic();
      /// @brief Converts map coordinate values to base coordinate values using current robot position in map coordinate. Be careful as inteded values may change if robot moves in map coordinates.
      /// @param[in] _x x value in map coordinate.
      /// @param[in] _y y value in map coordinate.
      /// @param[in] _z z value in map coordinate.
      /// @return Converted base value, valid as long as robot is in same position.
    public: Vector3 volatileTransformToBase(double _x, double _y, double _z);

      /// @brief set the value to robot model's hand angle
      /// @param[in] _arm aero::arm::(rarm|larm)
      /// @param[in] _radian target radian
    public: virtual void setHand(aero::arm _arm, double _radian);
    protected: virtual void setHandsFromJointStates_();

      /// @brief update the model's link poses based on angle values
    public: void updateLinkTransforms();

      // ------------------------------------------------------------
      // set modes
      // ------------------------------------------------------------
      /// @brief change the interpolation of angle execution
      /// @brief _i_type the list is in aero_std/include/interpolation_type.h
    public: bool setInterpolation(int _i_type);

      /// @brief This function must be set to true if neck and body are moved in different threads (including within code threads). This function must be set to false manually when threads are joined or when tracking was disabled. The function is only a declaration of "neck will be moved in different threads" and has nothing to do with the actual tracking.
    public: void setTrackingMode(bool _yes);

      /// @brief add priority to lifter ik mode. when in OnPlane mode, lifter moves x and z but z is limited. (this method is deprecated)
      /// @attention when you call setFromIK with aero:ikrange::lifter, both modes are tried but the one with has priority is used previously.
    public: void switchOnPlane();
      /// @brief add priority to lifter ik mode. when in HeightOnly mode, lifter moves only z but z is full range. (this method is deprecated)
      /// @attention when you call setFromIK with aero:ikrange::lifter, both modes are tried but the one with has priority is used previously.
    public: void switchHeightOnly();

      // ------------------------------------------------------------
      // get robot model's states
      // ------------------------------------------------------------
      /// @brief get joint angles from robot model
      /// @param[out] _av joint angles vector
    public: void getRobotStateVariables(std::vector<double> &_av);
      /// @brief get joint angles from robot model
      /// @param[out] _map joint angles map
    public: void getRobotStateVariables(std::map<std::string, double> &_map);
      /// @brief get joint angles from robot model, recommend
      /// @param[out] _map joint angles map
    public: void getRobotStateVariables(aero::joint_angle_map &_map);
      /// @brief get joint angles from robot model including hand angles
      /// @param[out] _map joint angles map
    public: void getRobotStateVariables(aero::fullarm &_map);

      /// @brief get named target "reset-pose", its basic pose of robot
      /// @param[out] _map joint angles map
    public: void getResetManipPose(aero::joint_angle_map &_map);

      /// @brief get waist position in base_link coordinate in robot model
      /// @return waist position
    public: Vector3 getWaistPosition();
      /// @brief get lifter relative position from top of the lifter in robot model
    public: void getLifter(aero::joint_angle_map& _xz);

      /// @brief get hand angle in robot model
      /// param[in] _arm which arm aero::arm::(rarm|larm)
      /// @return hand angle
    public: virtual double getHand(aero::arm _arm);

      /// @brief get end effector position
      /// @param[in] _arm arm which eef is fixed to
      /// @param[in] _eef aero::eef::(hand|grasp|pick|index|thumb)
      /// @return eef's position in base_link coordinate
    public: Vector3 getEEFPosition(aero::arm _arm, aero::eef _eef);
      /// @brief get end effector orientation
      /// @param[in] _arm arm which eef is fixed to
      /// @param[in] _eef aero::eef::(hand|grasp|pick|index|thumb)
      /// @return eef's quaternion in base_link coordinate
    public: Quaternion getEEFOrientation(aero::arm _arm, aero::eef _eef);

      /// @brief get move group for moveit
      /// @param[in] _move_group move group name
      /// @return desired move group
    public: AeroMoveGroup &getMoveGroup(std::string _move_group);
      /// @brief get move group for moveit
      /// @param[in] _arm arm which desired move group is associated
      /// @param[in] _range arm only, with torso, and with lifter are usable
      /// @return desired move group
    public: AeroMoveGroup &getMoveGroup(aero::arm _arm, aero::ikrange _range);

      // ------------------------------------------------------------
      // send to real robot
      // ------------------------------------------------------------
      /// @brief send reset-pose to real robot, simultaneously set reset-pose to model too.
      /// @param[in] _time_ms execution time, and wait this time
    public: void sendResetManipPose(int _time_ms=3000);

      /// @brief send joint angles in robot model to real robot
      /// @attention joints using are determined by args. for example when sendAnglevector(aero::arm::rarm, aero:ikrange::lifter, 3000) is called,
      /// upperbody without left arm will move
      /// @param[in] _arm witch arm to use
      /// @param[in] _range use arm only , with torso, or with lifter aero::ikrange::(arm|torso|lifter)
      /// @param[in] _time_ms execution time, and wait this time
    public: void sendAngleVector(aero::arm _arm, aero::ikrange _range, int _time_ms, bool _async=false); // _av in kinematic_state is used
      /// @brief send joint angles in robot model to real robot
      /// @attention use all joints on upper body
      /// @param[in] _time_ms execution time, and wait this time
      /// @param[in] _move_waist if it's aero::ikrange::lifter, the lifter will move
    public: void sendAngleVector(int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso, bool _aync=false); // all angles from kinematic_state is published
      /// @brief send joint angles in _av_map to real robot (this method is deprecated)
      /// @param[in] _av_map map which has joint name and joint angle value
      /// @param[in] _time_ms execution time, and wait this time
      /// @param[in] _move_waist if it's aero::ikrange::lifter, the lifter will move
    public: void sendAngleVector(aero::joint_angle_map _av_map, int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso);
      /// @brief send joint angles including hand angles in _av_map to real robot (this method is deprecated)
      /// @param[in] _av_map map which has joint name and joint angle value
      /// @param[in] _time_ms execution time, and wait this time
      /// @param[in] _move_waist if it's aero::ikrange::lifter, the lifter will move
    public: void sendAngleVector(aero::fullarm _av_map, int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso);

      /// @brief send joint angles in robot model to real robot  (this method is deprecated)
      /// @param[in] _arm which arm to use
      /// @param[in] _range use arm only , with torso, or with lifter aero::ikrange::(arm|torso|lifter)
      /// @param[in] _time_ms execution time, this function returns soon after called
    public: void sendAngleVectorAsync(aero::arm _arm, aero::ikrange _range, int _time_ms); // _av in kinematic_state is used
      /// @brief send joint angles in robot model to real robot  (this method is deprecated)
      /// @attention use all joints on upper body
      /// @param[in] _time_ms execution time, this function returns soon after called
      /// @param[in] _move_waist if it's aero::ikrange::lifter, the lifter will move
    public: void sendAngleVectorAsync(int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso); // all angles from kinematic_state is published
      /// @brief send joint angles in _av_map to real robot  (this method is deprecated)
      /// @param[in] _av_map map which has joint name and joint angle value
      /// @param[in] _time_ms execution time, this function returns soon after called
      /// @param[in] _move_waist if it's aero::ikrange::lifter, the lifter will move
    public: void sendAngleVectorAsync(aero::joint_angle_map _av_map, int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso);
      /// @brief send joint angles including hand angles in _av_map to real robot  (this method is deprecated)
      /// @param[in] _av_map map which has joint name and joint angle value
      /// @param[in] _time_ms execution time, this function returns soon after called
      /// @param[in] _move_waist if it's aero::ikrange::lifter, the lifter will move
    public: void sendAngleVectorAsync(aero::fullarm _av_map, int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso);

      /// @brief send joints trajectory to real robot
      /// @attention trajectory type is std::vector<aero::joint_angle_map>
      /// @param[in] _trajectory joints trajectory will be executed
      /// @param[in] _times execution time. the size of times vector need to be equal to the size of trajectory
      /// @param[in] _move_lifter if it's aero::ikrange::lifter, the lifter will move
      /// @return when times.size is not equal to trajectory.size, return false
    public: bool sendTrajectory(aero::trajectory _trajectory, std::vector<int> _times, aero::ikrange _move_lifter=aero::ikrange::torso, bool _async = false);
      /// @brief send joints trajectory to real robot
      /// @param[in] _trajectory joints trajectory will be executed
      /// @param[in] _time_ms split this time to trajectory size and execute trajectory on each splitted times
      /// @param[in] _move_lifter if it's aero::ikrange::lifter, the lifter will move
    public: bool sendTrajectory(aero::trajectory _trajectory, int _time_ms, aero::ikrange _move_lifter=aero::ikrange::torso, bool _async = false);

      /// @brief send joints trajectory to real robot (this method is deprecated)
      /// @param[in] _trajectory joints trajectory will be executed
      /// @param[in] _times execution time. the size of times vector need to be equal to the size of trajectory
      /// @param[in] _move_lifter if it's aero::ikrange::lifter, the lifter will move
      /// @return when times.size is not equal to trajectory.size, return false
    public: bool sendTrajectoryAsync(aero::trajectory _trajectory, std::vector<int> _times, aero::ikrange _move_lifter=aero::ikrange::torso);
      /// @brief send joints trajectory to real robot (this method is deprecated)
      /// @param[in] _trajectory joints trajectory will be executed
      /// @param[in] _time_ms split this time to trajectory size and execute trajectory on each splitted times
      /// @param[in] _move_lifter if it's aero::ikrange::lifter, the lifter will move
    public: bool sendTrajectoryAsync(aero::trajectory _trajectory, int _time_ms, aero::ikrange _move_lifter=aero::ikrange::torso);

      /// @brief protected function. the base function of sendAngleVectorAsync
    protected: void sendAngleVectorSync_(int _time_ms);
    protected: void sendAngleVectorAsync_(int _time_ms, aero::ikrange _move_waist); // _av in kinematic_state is used
    protected: void sendAngleVectorAsync_(std::string _move_group, int _time_ms); // _av in kinematic_state is used
    protected: void sendAngleVectorAsync_(const std::vector<double> _av, const std::vector<std::string> _joint_names, const int _time_ms);

      // @brief overwrite command speed on real robot
      // @param[in] _speed_factor < 1.0 for slow down, > 1.0 for speed up
    public: void overwriteSpeed(float _speed_factor);

      /// @brief send lifter position to real robot
      /// @attention when lifter is initial position (stretched), x and z are zero.
      /// @param[in] _x desired x position in meters
      /// @param[in] _z desired z position in meters
      /// @param[in] _time_ms execution time, and sleep this time before this function returns
    public: bool sendLifter(double _x, double _z, int _time_ms=5000, bool _local=false, bool _async=false); // m
      /// @brief send lifter position to real robot  (this method is deprecated)
      /// @param[in] _x desired x position in mili meters
      /// @param[in] _z desired z position in mili meters
      /// @param[in] _time_ms execution time, and sleep this time before this function returns
    public: bool sendLifter(int _x, int _z, int _time_ms=5000); // mm deprecated
      /// @brief send lifter position to real robot  (this method is deprecated)
      /// @param[in] _x desired relative x position in meters
      /// @param[in] _z desired relative z position in meters
      /// @param[in] _time_ms execution time, and sleep this time before this function returns
    public: bool sendLifterLocal(double _x, double _z, int _time_ms=5000);
      /// @brief send lifter position to real robot  (this method is deprecated)
      /// @param[in] _x desired relative x position in mili meters
      /// @param[in] _z desired relative z position in mili meters
      /// @param[in] _time_ms execution time, and sleep this time before this function returns
    public: bool sendLifterLocal(int _x, int _z, int _time_ms=5000);
      /// @brief send lifter position to real robot  (this method is deprecated)
      /// @param[in] _x desired x position in meters
      /// @param[in] _z desired z position in meters
      /// @param[in] _time_ms execution time, this function returns soon after called
    public: bool sendLifterAsync(double _x, double _z, int _time_ms=5000); // m
      /// @brief send lifter position to real robot  (this method is deprecated)
      /// @param[in] _x desired x position in mili meters
      /// @param[in] _z desired z position in mili meters
      /// @param[in] _time_ms execution time, this function returns soon after called
    public: bool sendLifterAsync(int _x, int _z, int _time_ms=5000); // mm  deprecated
      /// @brief send lifter position to real robot (this method is deprecated)
      /// @param[in] _x desired relative x position in meters
      /// @param[in] _z desired relative z position in meters
      /// @param[in] _time_ms execution time, this function returns soon after called
    public: bool sendLifterLocalAsync(double _x, double _z, int _time_ms=5000);
      /// @brief send lifter position to real robot (this method is deprecated)
      /// @param[in] _x desired relative x position in mili meters
      /// @param[in] _z desired relative z position in mili meters
      /// @param[in] _time_ms execution time, this function returns soon after called
    public: bool sendLifterLocalAsync(int _x, int _z, int _time_ms=5000);

      /// @brief cancel async lifter move (position stays where cancel was called)
    public: bool cancelLifter();

      /// @brief send lifter trajectory to real robot (this method is deprecated)
      /// @param[in] _trajectory desired lifter trajectory. x and z pair in meters
      /// @param[in] _times execution time. the size of times vector needs to be equal to the size of trajectory
    public: bool sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times);
      /// @brief send lifter trajectory to real robot (this method is deprecated)
      /// @param[in] _trajectory desired lifter trajectory. x and z pair in meters
      /// @param[in] _time_ms split this time to trajectory size and execute trajectory on each splitted times
    public: bool sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, int _time_ms);
      /// @brief send lifter trajectory to real robot (this method is deprecated)
      /// @param[in] _trajectory desired lifter trajectory. x and z pair in meters
      /// @param[in] _times execution time. the size of times vector needs to be equal to the size of trajectory
    public: bool sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times);
      /// @brief send lifter trajectory to real robot (this method is deprecated)
      /// @param[in] _trajectory desired lifter trajectory. x and z pair in meters
      /// @param[in] _time_ms split this time to trajectory size and execute trajectory on each splitted times
    public: bool sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, int _time_ms);

      /// @brief wait until trajecory execution finishes.
      /// use with send{AngleVector|Trajectory|Lifter}Async
      /// @param[in] _timeout_ms if waiting talkes longer than this time, the method returns
      /// if _timeout_ms == 0, timeout will not occur.
      /// sleeps a little before call waitInterpolation_ to guarantee the controller state
      /// @return if timeout occurs, returns false
    public: bool waitInterpolation(int _timeout_ms=0);
      /// @brief prototype for waitInterpolation
    protected: bool waitInterpolation_(int _timeout_ms=0);

      /// @brief waitInterpolation_ for wait off mode
    protected: void sleepInterpolation(int _time_ms);

      /// @brief send grasp command to real robot
      /// @param[in] _arm aero::arm::(rarm|larm)
      /// @param[in] _power grasp power from 0\% to 100\%
    public: bool sendGrasp(aero::arm _arm, int _power=100);
      /// @brief send grasp command to real robot (automatically opens when fail detected)
      /// @param[in] _arm aero::arm::(rarm|larm)
      /// @param[in] _power grasp power from 0\% to 100\%
      /// @param[in] _power fail angle in radian from 0.0 to 0.9
    public: bool sendGraspFast(aero::arm _arm, int _power=100, float _thre_fail=0.0);
      /// @brief open real robot's hand
      /// @param[in] _arm aero::arm::(rarm|larm)
    public: bool openHand(aero::arm _arm);
      /// @brief send desired hand angle to real robot
      /// @param[in] _arm aero::arm::(rarm|larm)
      /// @param[in] _rad desired angle in radian
    public: bool sendHand(aero::arm _arm, double _rad);
      /// @brief protected function calling HandControl service
      /// @param[in] _arm aero::arm::(rarm|larm)
      /// @param[in] _srv
    protected: bool callHandSrv_(const aero::arm &_arm, aero_startup::HandControl &_srv);

    public: bool solveIKSequence(aero::GraspRequest &_grasp);
    public: std::string solveIKOneSequence(aero::arm _arm, geometry_msgs::Pose _pose, aero::ikrange _ik_range, std::vector<double> _av_ini, std::string _eef_link, std::vector<double> &_result);
    public: bool sendSequence(std::vector<int> _msecs={5000, 5000});
    public: bool sendPickIK(aero::GraspRequest &_grasp);
    public: bool sendPlaceIK(aero::GraspRequest &_grasp, double _push_height=0.03);

      // ------------------------------------------------------------
      // move_base functions
      // ------------------------------------------------------------
      /// @brief get current robot's pose in world
      /// @param[in] _map map name
      /// @return current pose from _map to base_link
    public: geometry_msgs::Pose getCurrentPose(std::string _map="/map");
      /// @brief get named location's pose
      /// @param[in] _location see spot.yaml
      /// @return pose from map to location
    public: geometry_msgs::Pose getLocationPose(std::string _location);
      /// @brief move wheel to relative position
      /// @param[in] _x relative x in meters
      /// @param[in] _y relative y in meters
      /// @param[in] _timeout_ms timeout
      /// @return if lifter can't reach the desired position before timeout, return false
    public: bool goPos(double _x, double _y, double _rad, int _timeout_ms=20000);
      /// @brief move wheel to relative position
      /// @attention this function returns sonn after called. you can get the information by calling isMoving
      /// @param[in] _x relative x in meters
      /// @param[in] _y relative y in meters
    public: void goPosAsync(double _x, double _y, double _rad);
      /// @brief move wheel to named location
      /// @attention this function returns sonn after called. you can get the information by calling isMoving
      /// @param[in] _location see spot.yaml
    public: void moveToAsync(std::string _location);
      /// @brief move wheel to desired position
      /// @attention this function returns sonn after called. you can get the information by calling isMoving
      /// @param[in] _point desired position in map coordinate
    public: void moveToAsync(Vector3 _point);
      /// @brief move wheel to desired pose
      /// @attention this function returns sonn after called. you can get the information by calling isMoving
      /// @param[in] _pose desired pose in map coordinate
    public: void moveToAsync(geometry_msgs::Pose _pose);
      /// @brief check asynchronous wheel function is on process or not
      /// @return when wheel is moving, return true
    public: bool isMoving();
      /// @brief check robot is at location or not
      /// @param[in] _location see spot.yaml
      /// @param[in] _thre threshold
      /// @return if base_link is inside thresold in map coordinate, return true
    public: bool at(std::string _location, double _thre=0.2);
      /// @brief check robot is at _pose or not
      /// @param[in] _pose desired pose
      /// @param[in] _thre threshold
      /// @return if base_link is inside thresold in map coordinate, return true
    public: bool at(geometry_msgs::Pose _pose, double _thre=0.2);
      /// @brief stop asynchronous wheel functions
    public: void stop();
      /// @brief re-execure stopped functions
    public: void go();
      /// @brief get the distance to location
      /// @param[in] _location see spot.yaml
      /// @return distance from base_link to location in meters
    public: float toDestination(std::string _location);
      /// @brief turn the robot toward location
      /// @param[in] _location see spot.yaml
    public: void faceTowardAsync(std::string _location);
      /// @brief turn the robot toward pose
      /// @param[in] target pose in map coordinate
    public: void faceTowardAsync(geometry_msgs::Pose _pose);
      /// @brief check global cost map wether the robot can make plan to go to the position or not
      /// @param[in] _pose desired pose in map coordinate
      /// @return if wheel moving plan to the pose can be made, return true
    public: bool checkMoveTo(geometry_msgs::Pose _pose);
      /// @brief protected function. due to move_base_pkg's bug, we use this
    protected: bool goPosTurnOnly_(double _rad, int _timeout_ms=20000);


      // these varables are to use moveit libralies
      // don't care
      /// basic
    protected: robot_model_loader::RobotModelLoader robot_model_loader_;
    public: robot_model::RobotModelPtr kinematic_model;// robot model
    public: robot_state::RobotStatePtr kinematic_state;
      // lifter height only
    protected: robot_model_loader::RobotModelLoader robot_model_loader_ho_;
    public: robot_model::RobotModelPtr kinematic_model_ho;
    public: robot_state::RobotStatePtr kinematic_state_ho;
      // lifter on plane
    protected: robot_model_loader::RobotModelLoader robot_model_loader_op_;
    public: robot_model::RobotModelPtr kinematic_model_op;
    public: robot_state::RobotStatePtr kinematic_state_op;

      // MoveGroup
    public: AeroMoveGroup larm;
    public: AeroMoveGroup larm_with_torso;
    public: AeroMoveGroup larm_with_lifter;
    public: AeroMoveGroup rarm;
    public: AeroMoveGroup rarm_with_torso;
    public: AeroMoveGroup rarm_with_lifter;
    public: AeroMoveGroup lifter;
    public: AeroMoveGroup upper_body;
    public: AeroMoveGroup torso;
    public: AeroMoveGroup head;

      // JointModelGroup
    public: const robot_state::JointModelGroup* jmg_larm;
    public: const robot_state::JointModelGroup* jmg_larm_with_torso;
    public: const robot_state::JointModelGroup* jmg_larm_with_lifter;
    public: const robot_state::JointModelGroup* jmg_larm_with_lifter_ho;
    public: const robot_state::JointModelGroup* jmg_larm_with_lifter_op;
    public: const robot_state::JointModelGroup* jmg_rarm;
    public: const robot_state::JointModelGroup* jmg_rarm_with_torso;
    public: const robot_state::JointModelGroup* jmg_rarm_with_lifter;
    public: const robot_state::JointModelGroup* jmg_rarm_with_lifter_ho;
    public: const robot_state::JointModelGroup* jmg_rarm_with_lifter_op;
    public: const robot_state::JointModelGroup* jmg_lifter;

      // planning scene interface
    public: moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      // moveit functions
    public: bool plan(std::string _move_group);
    public: bool execute();
    public: void viewTrajectory();
    public: void setStartStateToCurrentState(std::string _move_group);
    public: void setNamedTarget(std::string _move_group, std::string _target);
    public: bool move(std::string _move_group);

      // with linux_kinect
    public: void speakAsync(std::string _speech);
    public: void speak(std::string _speech, float _wait_sec);

      // waitInterpolation settings
    public: inline void disableWaitInterpolation() {wait_ = false;};
    public: inline void enableWaitInterpolation() {wait_ = true;};

      // callback functions
    protected: void JointStateCallback_(const sensor_msgs::JointState::ConstPtr &_msg);

    protected: ros::ServiceClient hand_grasp_client_;
    protected: ros::ServiceClient joint_states_client_;
    protected: ros::ServiceClient interpolation_client_;
    protected: ros::Publisher display_publisher_;
    protected: ros::Publisher angle_vector_publisher_;
    protected: ros::Publisher look_at_publisher_rpy_;
    protected: ros::Publisher look_at_publisher_base_;
    protected: ros::Publisher look_at_publisher_map_;
    protected: ros::Publisher look_at_publisher_base_static_;
    protected: ros::Publisher look_at_publisher_map_static_;
    protected: ros::Publisher speech_publisher_;
    protected: ros::Publisher cmd_vel_publisher_;
    protected: ros::Publisher lookat_target_publisher_;
    protected: ros::Publisher overwrite_speed_publisher_;
    protected: ros::Subscriber joint_states_subscriber_;
    protected: ros::ServiceClient waist_service_;
    protected: ros::ServiceClient lifter_ik_service_;
    protected: ros::ServiceClient send_angle_service_;
    protected: ros::ServiceClient get_spot_;
    protected: ros::ServiceClient check_move_to_;
    protected: ros::ServiceClient get_saved_neck_positions_;
    protected: actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_;
    protected: AeroMoveGroup::Plan plan_;
    protected: std::string planned_group_;
    protected: bool height_only_;
    protected: std::vector<std::vector<double>> trajectory_;
    protected: std::vector<std::string> trajectory_groups_;
    protected: sensor_msgs::JointState joint_states_;
    protected: double lifter_thigh_link_;// lifter's upper link
    protected: double lifter_foreleg_link_;// lifter's lower link
      /// @brief flag of whether neck will be controlled by different thread or node
    protected: bool tracking_mode_flag_;
    protected: aero_startup::AeroSendJoints send_joints_srv_;
    protected: tf::TransformListener listener_;
    protected: geometry_msgs::Pose pose_using_;
    protected: std::string lookat_topic_;
    protected: std::string previous_topic_;
    protected: bool wait_;
      /// @brief used for re-enabling wait_ in setTrackingMode(false)
    protected: bool saved_wait_settings_;

      /// @brief for handling speed overwrite on synchronous w/o wait
    protected: std::mutex so_mutex_;
    protected: float so_factor_;
    protected: float so_retime_scale_;
    protected: bool so_update_;

    protected: ros::ServiceClient in_action_service_;

    };
    typedef std::shared_ptr<AeroMoveitInterface> AeroMoveitInterfacePtr;
  }
}
#endif
