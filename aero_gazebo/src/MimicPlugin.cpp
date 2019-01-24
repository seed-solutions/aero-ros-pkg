#include "MimicPlugin.h"

namespace gazebo {
  namespace mimicplugin {
    GZ_REGISTER_MODEL_PLUGIN(MimicPlugin);
  }
}

void gazebo::mimicplugin::MimicPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  model = _parent;
  world = _parent->GetWorld();

  std::cerr << "Loading mimic plugin" << std::endl;
  if (_sdf->HasElement("mimic")) {
    //std::cerr << "has mimic" << std::endl;
    //std::cerr << _sdf->ToString("aa") << std::endl;
    sdf::ElementPtr el = _sdf->GetElement("mimic");

    while (!!el) {
      //std::cerr << "in while" << std::endl;
      //std::cerr << el->ToString("bb") << std::endl;
      std::string target_name = el->Get<std::string> ();
      //std::cerr << "name: " << target_name << std::endl;
      std::string source_name;
      double offset = 0;
      double multiplier = 1.0;

      sdf::ParamPtr p_jn = el->GetAttribute("joint");
      if (!!p_jn) {
        p_jn->Get(source_name);
      } else if ( el->HasElement("joint") ) {
        sdf::ElementPtr pt = el->GetElement("joint");
        if (!!pt) {
          source_name = pt->Get<std::string > ();
        }
      }

      if (source_name.size() > 0) {
        //std::cerr << "mimic: " << source_name << std::endl;
        sdf::ParamPtr p_off = el->GetAttribute("offset");
        if (!!p_off) {  p_off->Get(offset); }
        else if ( el->HasElement("offset") ) {
          sdf::ElementPtr pt = el->GetElement("offset");
          if (!!pt) {
            offset = pt->Get<double > ();
          }
        }
        //
        sdf::ParamPtr p_mlt = el->GetAttribute("multiplier");
        if (!!p_mlt) {  p_mlt->Get(multiplier); }
        else if ( el->HasElement("multiplier") ) {
          sdf::ElementPtr pt = el->GetElement("multiplier");
          if (!!pt) {
            multiplier = pt->Get<double > ();
          }
        }
        // create mimic
        gazebo::mimicplugin::PidParams param;
        {
          sdf::ParamPtr attr = el->GetAttribute("P");
          if (!!attr) {  attr->Get(param.p); }
          else if ( el->HasElement("P") ) {
            sdf::ElementPtr pt = el->GetElement("P");
            if (!!pt) {
              param.p = pt->Get<double > ();
            }
          }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("I");
          if (!!attr) {  attr->Get(param.i); }
          else if ( el->HasElement("I") ) {
            sdf::ElementPtr pt = el->GetElement("I");
            if (!!pt) {
              param.i = pt->Get<double > ();
            }
          }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("D");
          if (!!attr) {  attr->Get(param.d); }
          else if ( el->HasElement("D") ) {
            sdf::ElementPtr pt = el->GetElement("D");
            if (!!pt) {
              param.d = pt->Get<double > ();
            }
          }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("i_max");
          if (!!attr) {  attr->Get(param.i_max); }
          else if ( el->HasElement("i_max") ) {
            sdf::ElementPtr pt = el->GetElement("i_max");
            if (!!pt) {
              param.i_max = pt->Get<double > ();
            }
          }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("i_min");
          if (!!attr) {  attr->Get(param.i_min); }
          else if ( el->HasElement("i_min") ) {
            sdf::ElementPtr pt = el->GetElement("i_min");
            if (!!pt) {
              param.i_min = pt->Get<double > ();
            }
          }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("command_max");
          if (!!attr) {  attr->Get(param.cmd_max); }
          else if ( el->HasElement("command_max") ) {
            sdf::ElementPtr pt = el->GetElement("command_max");
            if (!!pt) {
              param.cmd_max = pt->Get<double > ();
            }
          }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("command_min");
          if (!!attr) {  attr->Get(param.cmd_min); }
          else if ( el->HasElement("command_min") ) {
            sdf::ElementPtr pt = el->GetElement("command_min");
            if (!!pt) {
              param.cmd_min = pt->Get<double > ();
            }
          }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("velocity");
          if (!!attr) {  attr->Get(param.velocity); }
          else if ( el->HasElement("velocity") ) {
            sdf::ElementPtr pt = el->GetElement("velocity");
            if (!!pt) {
              param.velocity = pt->Get<bool > ();
            }
          }
        }
        registerMimic(source_name, target_name, offset, multiplier, param);
      } else {
        // warn
      }
      el = el->GetNextElement("mimic");
    }
  }
#if GAZEBO_MAJOR_VERSION < 8
  prev_tm = world->GetSimTime();
#else
  prev_tm = world->SimTime();
#endif
  updateConnection =
    event::Events::ConnectWorldUpdateBegin(
      boost::bind(&gazebo::mimicplugin::MimicPlugin::Update, this));
}

void gazebo::mimicplugin::MimicPlugin::registerMimic(const std::string &src_joint,
                                                     const std::string &dst_joint,
                                                     double offset, double multiplier, const PidParams &_p)
{
  std::cerr << "reg: " << src_joint << " -> " << dst_joint << std::endl;
  std::cerr << "offset: " << offset << ", multiplier: " << multiplier << std::endl;

  gazebo::physics::JointPtr jp_source;
  gazebo::physics::JointPtr jp_target;
  jp_source = model->GetJoint(src_joint);
  jp_target = model->GetJoint(dst_joint);
  if (!jp_source || !jp_target) {
    // warn
    return;
  }
  MimicJointUpdater mimic(jp_source, jp_target, offset, multiplier, _p);
  mimic_joint_list.push_back(mimic);
}

void gazebo::mimicplugin::MimicPlugin::Update()
{
  //std::cerr << "up" << std::endl;
#if GAZEBO_MAJOR_VERSION < 8
  gazebo::common::Time dt = world->GetSimTime() - prev_tm;
#else
  gazebo::common::Time dt = world->SimTime() - prev_tm;
#endif
  for(auto it : mimic_joint_list) {
    it.update(dt);
  }
}
