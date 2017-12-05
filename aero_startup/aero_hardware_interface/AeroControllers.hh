#ifndef AERO_CONTROLLER_AERO_CONTROLLERS_H_
#define AERO_CONTROLLER_AERO_CONTROLLERS_H_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <string>
#include <stdint.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include "aero_hardware_interface/Constants.hh"
#include "aero_hardware_interface/AJointIndex.hh"
#include "aero_hardware_interface/AeroControllerProto.hh"

namespace aero
{
  namespace controller
  {

  /// @brief Upper body controller
  ///
  /// The most part of this class is defined in AeroControllerProto,
  /// please reffer it.
    class AeroUpperController : public AeroControllerProto
    {
      /// @brief constructor
      /// @param _port USB port file name
    public: AeroUpperController(const std::string& _port);

      /// @brief destructor
    public: ~AeroUpperController();

    /// @brief utility servo on command
    public: void util_servo_on();

    /// @brief utility servo off command
    public: void util_servo_off();

    /// @brief hand_script command
    public: void Hand_Script(uint16_t _sendnum, uint16_t _script);

    };

  /// @brief Lower body controller
  ///
  /// This class has some functions related to wheel control
    class AeroLowerController : public AeroControllerProto
    {
    public: AeroLowerController(const std::string& _port);

    public: ~AeroLowerController();

    /// @brief servo on command, wheels will servo off
    public: void servo_on();

    /// @brief servo off command
    public: void servo_off();

    /// @brief servo on command including wheel
    ///   if you want to servo off only wheel, call servo_on()
    public: void wheel_on();

    /// @brief servo off command only wheel
    public: void wheel_only_off();

    /// @brief servo toggle command with wheels
    /// @param _d0 joints 1: on, 0: off
    /// @param _d1 wheels 1: on, 0: off
    protected: void servo_command(int16_t _d0, int16_t _d1);

    public: int32_t get_wheel_id(std::string& _name);

    public: std::string get_wheel_name(size_t _idx);

    public: std::vector<int16_t>& get_reference_wheel_vector();

    /// @brief set wheel velocity
    public: void set_wheel_velocity(std::vector<int16_t>& _wheel_vector,
				    uint16_t _time);

    protected:
      std::vector<int16_t> wheel_vector_;
      std::vector<int16_t> wheel_ref_vector_;
      std::vector<int16_t> wheel_cur_vector_;

      std::vector<AJointIndex> wheel_indices_;

      bool wheel_servo_;
    };

  }
}

#endif
