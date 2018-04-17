#include <string>
#include <vector>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

#include <gazebo/math/gzmath.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/ContactSensor.hh>
#include <gazebo/sensors/ImuSensor.hh>
#include <gazebo/sensors/Sensor.hh>

namespace gazebo
{
  namespace mimicplugin
  {
    struct PidParams {
    public:
      double p;
      double i;
      double d;
      double i_max;
      double i_min;
      double cmd_max;
      double cmd_min;
      PidParams () : p(100), i(0), d(0.1), i_max(0), i_min(0), cmd_max(1000), cmd_min(-1000) { }
      PidParams (double _p, double _i, double _d,
                 double _imax, double _imin, double _cmax, double _cmin) :
        p(_p), i(_i), d(_d), i_max(_imax), i_min(_imin), cmd_max(_cmax), cmd_min(_cmin)
      { }
    };

    class MimicJointUpdater {
    public:
      MimicJointUpdater(gazebo::physics::JointPtr source, gazebo::physics::JointPtr target,
                        double _offset, double _multiplier, const PidParams &_param) {
        source_joint = source;
        target_joint = target;
        offset = _offset;
        multiplier = _multiplier;
        // pid setting
        setPID(_param);
      }

      void setPID(const PidParams &p)
      {
        setPID(p.p, p.i, p.d, p.i_max, p.i_min, p.cmd_max, p.cmd_min);
      }
      void setPID(double _p, double _i, double _d,
                  double _imax, double _imin, double _cmax, double _cmin)
      {
        std::cerr << "P: " << _p
                  << ", I: " << _i
                  << ", D: " << _d
                  << ", imax: " << _imax
                  << ", imin: " << _imin
                  << ", cmax: " << _cmax
                  << ", cmin: " << _cmin << std::endl;
        pid.Init(_p, _i, _d,   _imax, _imin,  _cmax, _cmin);
      }
      void update (gazebo::common::Time &dt) {
        double t_current = target_joint->GetAngle(0).Radian();
        double s_current = source_joint->GetAngle(0).Radian();
        double t_desired = (s_current - offset) * multiplier;

        double result = pid.Update(t_current - t_desired, dt);
#if 0
        std::cerr << "t_cur: " << t_current
                  << ", s_cur: " << s_current
                  << ", t_tgt: " << t_desired
                  << ", err: " << t_current - t_desired
                  << ", r= " << result;
#endif
        // target_joint->SetVelocity(0, j_velocity); // velocity feedback
        target_joint->SetForce(0, result);
      }
    private:
      gazebo::physics::JointPtr source_joint;
      gazebo::physics::JointPtr target_joint;
      double offset;
      double multiplier;
      gazebo::common::PID pid;
    };

    class MimicPlugin : public ModelPlugin
    {
    public:
      MimicPlugin() {}
      virtual ~MimicPlugin() {}

      //
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    private:
      void Update();

      void registerMimic(const std::string &src_joint,
                         const std::string &dst_joint,
                         double offset, double multiplier,
                         const PidParams &_p);

      physics::WorldPtr world;
      physics::ModelPtr model;

      std::vector<MimicJointUpdater> mimic_joint_list;

      event::ConnectionPtr updateConnection;

      gazebo::common::Time prev_tm;
    };
  }
}
