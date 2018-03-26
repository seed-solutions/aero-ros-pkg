#ifndef _AERO_MOVEIT_INTERFACE_
#define _AERO_MOVEIT_INTERFACE_

#define USING_LOOKAT 1
#define USING_BASE   1
#define USING_HAND   1
#define USING_GRASP  1

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

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
#include <aero_startup/HandControl.h>
#include <aero_startup/AeroSendJoints.h>

// ros controller
#include <aero_std/AeroRobotInterface.hh>

#include <mutex>

#if USING_BASE
/// TODO: new code(move_base)
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>
#endif

namespace aero
{
  typedef std::vector<aero::joint_angle_map> trajectory;
  namespace interface
  {
    enum struct send_type : int {none, angles, sequence, stop_angles, stop_sequence};

    struct ControllerCommand
    {
    public:
      ros::Time start_time;
      aero::interface::send_type send_type;

      // for send_type == aero::interface::send_type::angles
      std::vector<double > angle_vector;
      std::vector<std::string > joint_names;
      double duration;

      // for send_type == aero::interface::send_type::sequence
      std::vector<robot_interface::angle_vector > angle_vector_sequence;
      std::vector<std::string > controller_names;
      std::vector<double > time_sequence;
    };

    class AeroMoveitInterface
    {
    public: typedef std::shared_ptr<AeroMoveitInterface> Ptr;

      /// @brief constructor
      /// @param[in] _nh ros node handler
      /// @param[in] _rd robot_description's name, "${_rd}", "${_rd}_ho" and "${_rd}_op" will be loaded
    public: explicit AeroMoveitInterface(ros::NodeHandle &_nh, const std::string &_rd="robot_description");
    public: ~AeroMoveitInterface();

      // ------------------------------------------------------------
      // set robot model's states
      // ------------------------------------------------------------

      /// @brief set robot model's angles
      /// @param[in] _av angle vector
    public: void setRobotStateVariables(const std::vector<double> &_av);
      /// @brief set robot model's angles
      /// param[in] _map map<joint name, radian>
    public: void setRobotStateVariables(const std::map<std::string, double> &_map);
      /// @brief set robot model's angles, recommend using this!
      /// @param[in] _map map<joint name, radian>
    public: void setRobotStateVariables(const aero::joint_angle_map &_map);

      /// @brief set current real robot's angles to robot model's angles
    public: void setRobotStateToCurrentState();
      /// @brief set named angles such as "reset-pose" to robot model's angles
      /// @param[in] _move_group named target is declared with move group
      /// @param[in] _target target name, target list is in .srdf file in aero_moveit_config
    public: void setRobotStateToNamedTarget(const std::string &_move_group, const std::string &_target);

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

#if USING_LOOKAT
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
#endif

      /// @brief update the model's link poses based on angle values
    public: void updateLinkTransforms();

      /// @brief This function must be set to true if neck and body are moved in different threads (including within code threads). This function must be set to false manually when threads are joined or when tracking was disabled. The function is only a declaration of "neck will be moved in different threads" and has nothing to do with the actual tracking.
    public: void setTrackingMode(bool _yes);

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

    public: void setPoseVariables(const aero::pose &_pose);
    public: void getPoseVariables(const aero::pose &_pose, aero::joint_angle_map &_map);

      /// @brief get named target "reset-pose", its basic pose of robot (deprecated)
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

      /// @brief get joint model group
      /// @param[in] _move_group move group name
      /// @return desired move group
    public: const robot_state::JointModelGroup *getJointModelGroup(const std::string &_move_group);
      /// @brief get joint model group
      /// @param[in] _arm arm which desired move group is associated
      /// @param[in] _range arm only, with torso, and with lifter are usable
      /// @return desired move group
    public: const robot_state::JointModelGroup *getJointModelGroup(aero::arm _arm, aero::ikrange _range);

      // ------------------------------------------------------------
      // send to real robot
      // ------------------------------------------------------------

      /// @brief send joint angles in robot model to real robot
      /// @attention joints using are determined by args. for example when sendAnglevector(aero::arm::rarm, aero:ikrange::lifter, 3000) is called,
      /// upperbody without left arm will move
      /// @param[in] _arm witch arm to use
      /// @param[in] _range use arm only , with torso, or with lifter aero::ikrange::(arm|waist|torso|lifter)
      /// @param[in] _time_ms execution time, and wait this time
    public: void sendAngleVector(aero::arm _arm, aero::ikrange _range, int _time_ms, bool _async=false); // _av in kinematic_state is used
      /// @brief send joint angles in robot model to real robot
      /// @attention use all joints on upper body
      /// @param[in] _time_ms execution time, and wait this time
      /// @param[in] _move_waist if it's aero::ikrange::lifter, the lifter will move
    public: void sendAngleVector(int _time_ms, aero::ikrange _move_waist=aero::ikrange::torso, bool _aync=false); // all angles from kinematic_state is published

      /// @brief send joints trajectory to real robot
      /// @attention trajectory type is std::vector<aero::joint_angle_map>
      /// @param[in] _trajectory joints trajectory will be executed
      /// @param[in] _times execution time. the size of times vector need to be equal to the size of trajectory
      /// @param[in] _move_lifter if it's aero::ikrange::lifter, the lifter will move
      /// @return when times.size is not equal to trajectory.size, return false
    public: bool sendTrajectory(const aero::trajectory &_trajectory, const std::vector<int> &_times,
                                aero::ikrange _move_lifter=aero::ikrange::torso, bool _async = false);
      /// @brief send joints trajectory to real robot
      /// @param[in] _trajectory joints trajectory will be executed
      /// @param[in] _time_ms split this time to trajectory size and execute trajectory on each splitted times
      /// @param[in] _move_lifter if it's aero::ikrange::lifter, the lifter will move
    public: bool sendTrajectory(const aero::trajectory &_trajectory, int _time_ms,
                                aero::ikrange _move_lifter=aero::ikrange::torso, bool _async = false);

      /// @brief protected function. the base function of sendAngleVectorAsync
    protected: void sendAngleVectorSync_(int _time_ms);
    protected: void sendAngleVectorAsync_(int _time_ms, aero::ikrange _move_waist); // _av in kinematic_state is used
    protected: void sendAngleVectorAsync_(const std::string &_move_group, int _time_ms); // _av in kinematic_state is used
    protected: void sendAngleVectorAsync_(const std::vector<double> &_av, const std::vector<std::string> &_joint_names, const int _time_ms);

      /// @brief send lifter position to real robot
      /// @attention when lifter is initial position (stretched), x and z are zero.
      /// @param[in] _x desired x position in meters
      /// @param[in] _z desired z position in meters
      /// @param[in] _time_ms execution time, and sleep this time before this function returns
    public: bool sendLifter(double _x, double _z, int _time_ms=5000, bool _local=false, bool _async=false); // m

      /// @brief cancel async lifter move (position stays where cancel was called)
    public: bool cancelLifter();

      /// @brief wait until trajecory execution finishes.
      /// use with send{AngleVector|Trajectory|Lifter}Async
      /// @param[in] _timeout_ms if waiting talkes longer than this time, the method returns
      /// if _timeout_ms == 0, timeout will not occur.
      /// sleeps a little before call waitInterpolation_ to guarantee the controller state
      /// @return if timeout occurs, returns false
    public: bool waitInterpolation(int _timeout_ms=0);
      /// @brief prototype for waitInterpolation
    protected: bool waitInterpolation_(int _timeout_ms=0);

#if USING_HAND // TODO devide code
      /// @brief set the value to robot model's hand angle
      /// @param[in] _arm aero::arm::(rarm|larm)
      /// @param[in] _radian target radian
    public: virtual void setHand(aero::arm _arm, double _radian);
    protected: virtual void setHandsFromJointStates_();

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
#endif

#if USING_GRASP // TODO device code / grasp ...
    public: bool solveIKSequence(const aero::GraspRequest &_grasp);
    public: bool solveIKOneSequence(aero::arm _arm, const aero::Transform &_pose, aero::ikrange _ik_range,
                                    const std::vector<double> &_av_initial, aero::eef _eef,
                                    std::string &_result_range, aero::joint_angle_map &_result);
    public: bool sendSequence(std::vector<int> _msecs={5000, 5000});
    public: bool sendPickIK(const aero::GraspRequest &_grasp);
    public: bool sendPlaceIK(const aero::GraspRequest &_grasp, double _push_height=0.03);
#endif

#if USING_BASE /// TODO: new code(move_base)
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
#endif

      // @brief overwrite command speed on real robot
      // @param[in] _speed_factor < 1.0 for slow down, > 1.0 for speed up
    public: void overwriteSpeed(float _speed_factor);

      // robot_model
    public: robot_model::RobotModelPtr kinematic_model;
    public: robot_state::RobotStatePtr kinematic_state;

      // JointModelGroup
    public: const robot_state::JointModelGroup* jmg_larm;
    public: const robot_state::JointModelGroup* jmg_larm_with_waist;
    public: const robot_state::JointModelGroup* jmg_larm_waist_lifter;

    public: const robot_state::JointModelGroup* jmg_rarm;
    public: const robot_state::JointModelGroup* jmg_rarm_with_waist;
    public: const robot_state::JointModelGroup* jmg_rarm_waist_lifter;

    public: const robot_state::JointModelGroup* jmg_waist;
    public: const robot_state::JointModelGroup* jmg_torso;
    public: const robot_state::JointModelGroup* jmg_lifter;
    public: const robot_state::JointModelGroup* jmg_both_arms;
    public: const robot_state::JointModelGroup* jmg_upper_body;
    public: const robot_state::JointModelGroup* jmg_whole_body;
    public: const robot_state::JointModelGroup* jmg_head;

    private: std::map<std::string, const robot_state::JointModelGroup* > joint_model_group_map;

#if USING_HAND
    protected: ros::ServiceClient hand_grasp_client_;
#endif

#if USING_LOOKAT
    protected: ros::Publisher look_at_publisher_rpy_;
    protected: ros::Publisher look_at_publisher_base_;
    protected: ros::Publisher look_at_publisher_map_;
    protected: ros::Publisher look_at_publisher_base_static_;
    protected: ros::Publisher look_at_publisher_map_static_;
    protected: ros::Publisher lookat_target_publisher_;

    protected: ros::ServiceClient get_saved_neck_positions_;

    protected: std::string lookat_topic_;
    protected: std::string previous_topic_;
#endif

#if USING_BASE
    protected: ros::Publisher cmd_vel_publisher_;
    protected: ros::ServiceClient get_spot_;
    protected: ros::ServiceClient check_move_to_;
    protected: actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_;

    protected: tf::TransformListener listener_;
    protected: geometry_msgs::Pose pose_using_;
#endif
    protected: bool tracking_mode_flag_;
    protected: aero::trajectory trajectory_;

    protected: boost::shared_ptr<aero::AeroRobotInterface > ri;
    protected: double send_trajectory_offset_;
    protected: ControllerCommand sent_command_;
    };
  }
}
#endif

#if 0
//// memo for renew

// remove moveit planning functions
// use only robot kinematics

// warn old_torso => waist
//  torso => lifter + wait

// remove ho, op kinematics (API not changed)

// remove speak

// for feature
// remove move base command
// remove hand command
#endif
