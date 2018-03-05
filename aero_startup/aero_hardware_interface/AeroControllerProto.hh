#ifndef AERO_CONTROLLER_AERO_CONTROLLER_PROTO_H_
#define AERO_CONTROLLER_AERO_CONTROLLER_PROTO_H_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>
#include <unordered_map>
#include <cmath>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include "aero_hardware_interface/CommandList.hh"
#include "aero_hardware_interface/Constants.hh"
#include "aero_hardware_interface/AJointIndex.hh"

using namespace boost::asio;

namespace aero
{
  namespace controller
  {
    /// @brief SEED controller via USB/RS485
    class SEED485Controller
    {
      /// @brief constructor
      /// @param _port USB port file name
      /// @param _id CAN bus ID
     public: SEED485Controller(const std::string& _port, uint8_t _id);

      /// @brief destructor
     public: ~SEED485Controller();

      /// @brief read from SEED controller
     public: void read(std::vector<uint8_t>& _read_data);

      /// @brief send command to SEED controller
      /// @param _cmd Command ID
      /// @param _time Destination time
      /// @param _send_data data buffer
     public: void send_command(uint8_t _cmd, uint16_t _time,
                               std::vector<uint8_t>& _send_data);

      /// @brief send single command to SEED controller
      /// @param _cmd Command ID
      /// @param _num Send ID
      /// @param _data data
     public: void send_command(uint8_t _cmd, uint8_t _num, uint16_t _data);

     public: void send_command(uint8_t _cmd, uint8_t _sub, uint16_t _time,
                               std::vector<uint8_t>& _send_data);

      /// @brief send_executing script command
     public: void AERO_Snd_Script(uint16_t sendnum, uint8_t scriptnum);

      /// @brief flush io buffer
     public: void flush();

      /// @brief send raw data to SEED controller
      /// @param _send_data raw data buffer
     public: void send_data(std::vector<uint8_t>& _send_data);

      /// @brief set / unset verbose mode
      /// @param val verbose mode
     public: void verbose(bool val) {verbose_ = val;}
      /// @brief get verbose mode flag
      /// @return verbose mode flag
     public: bool verbose() {return verbose_;}

      /// @brief getdebug mode flag
      /// @return true if in debug mode
     public: bool is_debug_mode() {return !ser_.is_open();}

     private: io_service io_;

     private: serial_port ser_;

     private: uint8_t id_;

     private: bool verbose_;

     private: boost::mutex mtx_;
    };  // SEED485Controller

    /// @brief super class of body controller,
    /// has SEED485Controller and some SEED command fucntions.
    class AeroControllerProto
    {
      /// @brief constructor
      /// @param _port USB port file name
      /// @param _id CAN bus ID
     public: AeroControllerProto(const std::string& _port, uint8_t _id);

      /// @brief destructor
     public: ~AeroControllerProto();

      /// @brief servo on command
     public: void servo_on();

      /// @brief servo off command
     public: void servo_off();

      /// @brief servo toggle command
      /// @param _d0 1: on, 0: off
     protected: void servo_command(int16_t _d0);

     public: std::vector<int16_t> get_reference_stroke_vector();

     public: std::vector<int16_t> get_actual_stroke_vector();

     public: std::vector<int16_t> get_status_vec();

     public: std::string get_stroke_joint_name(size_t _idx);

     public: int get_number_of_angle_joints();

     public: int get_number_of_strokes();

     public: int32_t get_ordered_angle_id(std::string _name);

     public: bool get_joint_name(int32_t _joint_id, std::string &_name);

     public: bool get_status();

     public: bool get_status(std::vector<bool>& _status_vector);

      /// @brief get current position from seed_
      ///   to access position externally, use get_actual_stroke_vector
     public: void update_position();

      /// @brief updates robot status (checks step out joints)
     public: void update_status();

     public: void reset_status();

      /// @brief send Get_Cur command
      /// @param _stroke_vector stroke vector
     public: void get_current(std::vector<int16_t>& _stroke_vector);

      /// @brief send Get_Tmp command
      /// @param _stroke_vector stroke vector
     public: void get_temperature(std::vector<int16_t>& _stroke_vector);

      /// @brief get data from buffer,
      ///   this does not call command, but only read from buffer
      /// @param _stroke_vector stroke vector
     protected: void get_data(std::vector<int16_t>& _stroke_vector);

      /// @brief abstract of get commands
      /// @param _cmd command id
      /// @param _stroke_vector stroke vector
     protected: void get_command(uint8_t _cmd,
                                 std::vector<int16_t>& _stroke_vector);

     protected: void get_command(uint8_t _cmd, uint8_t _sub,
                                 std::vector<int16_t>& _stroke_vector);

      /// @brief set position command
      /// @param _stroke_vector stroke vector, MUST be DOF bytes
      /// @param _time time[ms]
     public: void set_position(std::vector<int16_t>& _stroke_vector,
                               uint16_t _time);

      /// @brief send Motor_Cur command
      /// @param _stroke_vector stroke vector
     public: void set_max_current(std::vector<int16_t>& _stroke_vector);

      /// @brief send Motor_Cur command
      /// @param _num Send id
      /// @param _dat data
     public: void set_max_single_current(int8_t _num, int16_t _dat);

      /// @brief send Motor_Acc command
      /// @param _stroke_vector stroke vector
     public: void set_accel_rate(std::vector<int16_t>& _stroke_vector);

      /// @brief send Motor_Gain command
      /// @param _stroke_vector stroke vector
     public: void set_motor_gain(std::vector<int16_t>& _stroke_vector);

      /// @brief abstract of set commands
      /// @param _cmd command id
      /// @param _stroke_vector stroke vector
     protected: void set_command(uint8_t _cmd,
                                 std::vector<int16_t>& _stroke_vector);

      /// @brief stoke_vector to raw command bytes
     protected: void stroke_to_raw_(std::vector<int16_t>& _stroke,
                                    std::vector<uint8_t>& _raw);

     protected: bool verbose_;

     protected: boost::mutex ctrl_mtx_;

     protected: SEED485Controller seed_;

     protected: std::vector<int16_t> stroke_vector_;

     protected: std::vector<int16_t> stroke_ref_vector_;

     protected: std::vector<int16_t> stroke_cur_vector_;

     protected: std::vector<AJointIndex> stroke_joint_indices_;

     protected: std::vector<int16_t> status_vector_;

     protected: bool bad_status_;

     protected:
      std::unordered_map<std::string, int32_t> angle_joint_indices_;
    };  // AeroControllerProto

  /////////////////////
  // nonclass functions
  /////////////////////

  /// @brief decode short(int16_t) from byte(uint8_t)
  int16_t decode_short_(uint8_t* _raw);

  /// @brief ecnode short(int16_t) to byte(uint8_t)
  void encode_short_(int16_t _value, uint8_t* _raw);
  }
}

#endif
