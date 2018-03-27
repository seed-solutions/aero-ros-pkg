#ifndef AERO_GRASP_H_
#define AERO_GRASP_H_

#include <vector>
#include <string>
#include <fstream>

#include <ros/ros.h>

#include "aero_robot_hardware.h"

#include <aero_startup/GraspControl.h>

namespace aero
{
namespace grasp
{

class AeroGrasp
{
 public: typedef boost::shared_ptr< AeroGrasp> Ptr;

 public: AeroGrasp(const ros::NodeHandle& _nh,
                   aero_robot_hardware::AeroRobotHW *_in_hw) : nh_(_nh), hw_(_in_hw)
  {
    grasp_control_server_ =
      nh_.advertiseService(
        "grasp_control",
        &AeroGrasp::GraspControlCallback,
        this);
  }

 public: ~AeroGrasp() {} ;

 public: bool GraspControlCallback(aero_startup::GraspControl::Request&  _req,
                                   aero_startup::GraspControl::Response& _res) {
   // hw_->stopUpper(); // not needed

   ROS_WARN("AeroGrasp: Grasp pos: %d, script %d, power: %d",
            _req.position, _req.script, _req.power);

   ROS_WARN("AeroGrasp: setMaxSingleCurrent");
   hw_->setMaxSingleCurrent(_req.position, _req.power);

   usleep(200 * 1000); // 200ms sleep ???

   ROS_WARN("AeroGrasp: handScript");
   hw_->handScript(_req.position, _req.script);

   // return if cancel script
   if (_req.script == aero_startup::GraspControlRequest::SCRIPT_CANCEL) {
     ROS_WARN("AeroGrasp: End Grasp");
     // hw_->startUpper(); // not needed
     return true;
   }

   if(_req.script == aero_startup::GraspControlRequest::SCRIPT_GRASP) {
     usleep(3000 * 1000); // wait 3 seconds, must be 3!
   } else if(_req.script == aero_startup::GraspControlRequest::SCRIPT_UNGRASP) {
     usleep(1000 * 1000);
   }

   ROS_WARN("AeroGrasp: End Grasp");
   _res.angles.resize(2);
#if 0
   _res.angles[0] = upper_angles[13];
   _res.angles[1] = upper_angles[27];
#endif
   // hw_->startUpper(); // not needed
   return true;
 }

  /// @param node handle
 private: ros::NodeHandle nh_;

 private: ros::ServiceServer grasp_control_server_;
  ///
 private: aero_robot_hardware::AeroRobotHW *hw_;

};

}  // grasp
}  // aero

#endif
