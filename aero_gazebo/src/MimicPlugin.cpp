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

  //std::cerr << "mimic plugin" << std::endl;
  if (_sdf->HasElement("mimic")) {
    //std::cerr << "has mimic" << std::endl;
    //std::cerr << _sdf->ToString("aa") << std::endl;
    sdf::ElementPtr el = _sdf->GetElement("mimic");

    while (!!el) {
      //std::cerr << "in while" << std::endl;
      //std::cerr << el->ToString("bb") << std::endl;
      std::string target_name = el->Get<std::string> ();
      //std::cerr << "name: " << name << std::endl;
      std::string source_name;
      double offset = 0;
      double multiplier = 1.0;

      sdf::ParamPtr p_jn = el->GetAttribute("joint");
      if (!!p_jn) {
        p_jn->Get(source_name);
        //std::cerr << "mimic: " << jname;
        sdf::ParamPtr p_off = el->GetAttribute("offset");
        if (!!p_off) {  p_off->Get(offset); }
        //
        sdf::ParamPtr p_mlt = el->GetAttribute("multiplier");
        if (!!p_mlt) {  p_mlt->Get(multiplier); }
        // create mimic
        gazebo::mimicplugin::PidParams param;
        {
          sdf::ParamPtr attr = el->GetAttribute("P");
          if (!!attr) {  attr->Get(param.p); }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("I");
          if (!!attr) {  attr->Get(param.i); }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("D");
          if (!!attr) {  attr->Get(param.d); }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("i_max");
          if (!!attr) {  attr->Get(param.i_max); }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("i_min");
          if (!!attr) {  attr->Get(param.i_min); }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("command_max");
          if (!!attr) {  attr->Get(param.cmd_max); }
        }
        {
          sdf::ParamPtr attr = el->GetAttribute("command_min");
          if (!!attr) {  attr->Get(param.cmd_min); }
        }
        registerMimic(source_name, target_name, offset, multiplier, param);
      } else {
        // warn
      }
      el = el->GetNextElement("mimic");
    }
  }

  prev_tm = world->GetSimTime();

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
  gazebo::common::Time dt = world->GetSimTime() - prev_tm;
  for(auto it : mimic_joint_list) {
    it.update(dt);
  }
}
