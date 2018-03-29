namespace aero
{
  typedef std::vector<aero::joint_angle_map> trajectory;
  namespace interface
  {
#define DEP __attribute__((deprecated))
    ////// for backward compatibility
    class AeroMoveitInterfaceDeprecated : public AeroMoveitInterface
    {
    public: AeroMoveitInterfaceDeprecated(ros::NodeHandle &_nh,
                                          const std::string &_rd="robot_description")
      : AeroMoveitInterface(_nh, _rd)
      {
        ROS_WARN("This is deprecated class / AeroMoveitInterface");
      }
      //COMMON
    public: using AeroMoveitInterface::sendAngleVector;
    public: DEP void sendAngleVector(aero::arm _arm, aero::ikrange _range, int _time_ms, bool _async=true);
    public: DEP void sendAngleVector(int _time_ms, aero::ikrange _move_waist=aero::ikrange::upperbody, bool _async=true);

    public: using AeroMoveitInterface::getLifter;
    public: DEP void getLifter(aero::joint_angle_map &_xz);

    public: using AeroMoveitInterface::setFromIK;
    public: DEP bool setFromIK(std::string _move_group, geometry_msgs::Pose _pose, std::string _eef_link="", int _attempts=10);
    public: DEP bool setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, std::string _eef_link="", int _attempts=10);
    public: DEP bool setFromIK(aero::arm _arm, aero::ikrange _range, geometry_msgs::Pose _pose, aero::eef _eef, int _attempts=10);

    public: DEP bool setFromIK(std::string _move_group, Vector3 _pos, Quaternion _qua, std::string _eef_link="", int _attempts=10);
    public: DEP bool setFromIK(aero::arm _arm, aero::ikrange _range, Vector3 _pos, Quaternion _qua, std::string _eef_link="", int _attempts=10);
    public: DEP bool setFromIK(aero::arm _arm, aero::ikrange _range, Vector3 _pos, Quaternion _qua, aero::eef _eef, int _attempts=10);

    public: DEP void getResetManipPose(aero::joint_angle_map &_map);

    public: DEP void sendResetManipPose(int _time_ms=3000);

    public: DEP void sendAngleVectorAsync(aero::arm _arm, aero::ikrange _range, int _time_ms);
    public: DEP void sendAngleVectorAsync(int _time_ms, aero::ikrange _move_waist=aero::ikrange::upperbody);
    public: DEP void sendAngleVectorAsync(aero::joint_angle_map _av_map, int _time_ms, aero::ikrange _move_waist=aero::ikrange::upperbody);

// sendAngleVectorSequence
    public: DEP bool sendTrajectoryAsync(aero::trajectory _trajectory, std::vector<int> _times, aero::ikrange _move_lifter=aero::ikrange::upperbody);
    public: DEP bool sendTrajectoryAsync(aero::trajectory _trajectory, int _time_ms, aero::ikrange _move_lifter=aero::ikrange::upperbody);

    public: DEP bool sendLifter(double _x, double _z, int _time_ms=5000, bool _local=false, bool _async=false); // m
    public: DEP bool sendLifter(int _x, int _z, int _time_ms=5000);

    public: DEP bool sendLifterLocal(double _x, double _z, int _time_ms=5000);
    public: DEP bool sendLifterLocal(int _x, int _z, int _time_ms=5000);
    public: DEP bool sendLifterAsync(double _x, double _z, int _time_ms=5000);
    public: DEP bool sendLifterAsync(int _x, int _z, int _time_ms=5000);
    public: DEP bool sendLifterLocalAsync(double _x, double _z, int _time_ms=5000);
    public: DEP bool sendLifterLocalAsync(int _x, int _z, int _time_ms=5000);

    public: DEP bool cancelLifter();

      //???
    public: DEP bool sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times);
    public: DEP bool sendLifterTrajectory(std::vector<std::pair<double, double>>& _trajectory, int _time_ms);
    public: DEP bool sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, std::vector<int> _times);
    public: DEP bool sendLifterTrajectoryAsync(std::vector<std::pair<double, double>>& _trajectory, int _time_ms);
      // Grasp

      //BASE
    public: DEP geometry_msgs::Pose getCurrentPose(std::string _map="/map");
    public: DEP geometry_msgs::Pose getLocationPose(std::string _location);
      //public: bool goPos(double _x, double _y, double _rad, int _timeout_ms=20000);
    public: DEP void goPosAsync(double _x, double _y, double _rad);
    public: DEP void moveToAsync(std::string _location);
    public: DEP void moveToAsync(Vector3 _point);
    public: DEP void moveToAsync(geometry_msgs::Pose _pose);
      //public: bool isMoving();
    public: DEP bool at(std::string _location, double _thre=0.2);
    public: DEP bool at(geometry_msgs::Pose _pose, double _thre=0.2);
      //public: void stop();
      //public: void go();
      //public: float toDestination(std::string _location);
    public: DEP void faceTowardAsync(std::string _location);
    public: DEP void faceTowardAsync(geometry_msgs::Pose _pose);
    public: using AeroBaseCommander::checkMoveTo;
    public: DEP bool checkMoveTo(geometry_msgs::Pose _pose);
      //protected: bool goPosTurnOnly_(double _rad, int _timeout_ms=20000);
    };
    typedef boost::shared_ptr<AeroMoveitInterfaceDeprecated > AeroMoveitInterfacePtr;
  }
}
