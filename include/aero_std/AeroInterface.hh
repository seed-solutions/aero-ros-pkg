#ifndef _AERO_STD_AERO_INTERFACE_
#define _AERO_STD_AERO_INTERFACE_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "trajectory_msgs/JointTrajectory.h"
#include "aero_startup/AeroTorsoController.h"
#include "aero_startup/AeroHandController.h"
#include "aero_std/interpolation_type.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseFeedback.h>
#include <move_base_msgs/MoveBaseResult.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Vector3.h"
#include "aero_msgs/JointAngles.h"
#include "aero_msgs/GraspIK.h"
#include "aero_msgs/LookIK.h"
#include "aero_msgs/MBasedLoaded.h"
#include <mutex>

#include <tf/transform_broadcaster.h>

namespace aero
{
  namespace interface
  {

    class AeroInterface
    {
    public: explicit AeroInterface(ros::NodeHandle _nh);

    public: ~AeroInterface();

    public: void ResetManipPose();

    public: bool MoveWaist(Eigen::Vector3f _vector, std::string _coordinate);

    public: bool OpenHand(bool _yes, std::string _arm);

    public: bool OpenHand(bool _yes, std::string _arm, float _warn, float _fail);

    public: bool OpenHand(float _angle, std::string _arm);

    public: bool OpenHand(float _angle, std::string _arm, float _warn, float _fail);

    public: bool GoPos(float _x, float _y, float _theta);

    public: bool GoPos(float _x, float _y, float _theta, float _time_out);

    public: void SendAngleVector
    (aero_msgs::JointAngles _av, geometry_msgs::Vector3 _look_at,
     int _time_ms, bool _wait_interpolation=true);

    public: void SendAngleVector
    (aero_msgs::JointAngles _av, int _time_ms, bool _wait_interpolation=true);

    public: void SendAngleVector
    (geometry_msgs::Vector3 _look_at, int _time_ms, bool _wait_interpolation=true);

    public: void SendAngleVector
    (aero_msgs::JointAngles _av, geometry_msgs::Vector3 _look_at,
     int _time_ms, aero::interpolation::settings _intrpl,
     bool _wait_interpolation=true);

    public: void SendAngleVector
    (aero_msgs::JointAngles _av, int _time_ms,
     aero::interpolation::settings _intrpl, bool _wait_interpolation=true);

    public: void SendAngleVector
    (geometry_msgs::Vector3 _look_at, int _time_ms,
     aero::interpolation::settings _intrpl, bool _wait_interpolation=true);

    public: aero_msgs::JointAngles ReadOnlyResetManipPose();

    public: aero_msgs::GraspIK::Response ReadOnlyGraspIK
    (aero_msgs::GraspIK::Request _req);

    public: geometry_msgs::Vector3 ReadOnlyLookIK
    (Eigen::Vector3f _look_at, aero_msgs::JointAngles _av);

    // public: void LookAt(Eigen::Vector3f &_look_at);

    public: void StartGraspIKClient();

    public: void StartLookAtClient();

    public: aero_msgs::JointAngles ReverseJointAngles(aero_msgs::JointAngles& _av);

    private: bool MBasedLoaded(aero_msgs::MBasedLoaded::Request& _req,
                               aero_msgs::MBasedLoaded::Response& _res);

    private: void ConvertRad(aero_msgs::JointAngles& _av);

    private: ros::NodeHandle nh_;

    private: ros::Publisher pose_publisher_;

    private: ros::ServiceClient waist_client_;

    private: ros::ServiceClient hand_client_;

    private: ros::ServiceClient interpolation_client_;

    private: actionlib::SimpleActionClient
    <move_base_msgs::MoveBaseAction> *ac_;

    private: ros::ServiceClient grasp_client_;

    private: ros::ServiceClient look_client_;

    private: bool loaded_look_lib_;

    private: bool loaded_grasp_lib_;

    private: ros::ServiceServer mbased_loaded_;

    private: std::mutex ros_spin_mutex_;

    // human status

    public: inline void BeginListen() {
        std_msgs::String topic;
        topic.data = "/template/on";
        speech_detection_settings_publisher_.publish(topic);
      };

    public: inline void EndListen() {
        std_msgs::String topic;
        topic.data = "/template/off";
        speech_detection_settings_publisher_.publish(topic);
      };

    public: inline std::string Listen() {
      std::string result = detected_speech_;
      detected_speech_ = "";
      return result;
    };

    private: void Listener(const std_msgs::String::ConstPtr& _msg);

    private: ros::Subscriber speech_listener_;

    private: ros::Publisher speech_detection_settings_publisher_;

    private: std::string detected_speech_;

    // tts

    public: void Speak(std::string _speech);

    public: void SpeakAsync(std::string _speech);

    public: void Speak(std::string _speech, float _wait_sec);

    private: void TTSFlagListener(const std_msgs::String::ConstPtr& _msg);

    private: ros::Publisher speech_publisher_;

    private: ros::Subscriber tts_flag_listener_;

    private: bool tts_finished_;

    // robot status

    public: inline aero_msgs::JointAngles GetActualVector() { return body_pose_; };

    public: inline Eigen::Vector3f GetMoveWaist() { return base_position_world_; };

    private: aero_msgs::JointAngles body_pose_;

    private: Eigen::Vector3f base_position_world_;

    // optional

    public: void RotateKinectTo(float _angle);

    private: ros::Publisher kinect_control_publisher_;

    // PIMPL for viewer

    private: class AeroInterfaceImpl;

    private: std::shared_ptr<AeroInterfaceImpl> impl_;

    public: inline std::shared_ptr<AeroInterfaceImpl> GetImpl() { return impl_; };

    private: void InitViewer();

    private: void ViewGrasp(aero_msgs::GraspIK::Request _req);
    };

    typedef std::shared_ptr<AeroInterface> AeroInterfacePtr;

  }
}

#endif
