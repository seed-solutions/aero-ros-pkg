#ifndef AERO_CONTROLLER_AERO_CONTROLLER_PROTO_H_
#define AERO_CONTROLLER_AERO_CONTROLLER_PROTO_H_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include "aero_controller/Constants.hh"
#include "aero_controller/AJointIndex.hh"

using namespace boost::asio;

namespace aero
{
  namespace controller
  {

    class SEED485Controller
    {
    public: SEED485Controller(const std::string& _port, uint8_t _id);

    public: ~SEED485Controller();

    public: void read(std::vector<uint8_t>& _read_data);

    public: void send_command(uint8_t _cmd, uint16_t _time,
			      std::vector<uint8_t>& _send_data);

    /// @brief flush io buffer
    public: void flush();

    /// @brief create command header into data buffer
    /// @param _cmd command id
    /// @param _time time[ms]
    /// @param _dat data buffer, must be RAW_DATA_LENGTH(77) bytes.
    private: void set_command_header(uint8_t _cmd, uint16_t _time,
				    std::vector<uint8_t>& _dat);

    /// @brief calc checksum and put into last byte, MUST be called LAST.
    /// @param _dat data buffer, MUST be RAW_DATA_LENGTH(77) bytes.
    private: void set_check_sum(std::vector<uint8_t>& _dat);

    private: void send_data(std::vector<uint8_t>& _send_data);

    private: io_service io_;

    private: serial_port ser_;

    private: uint8_t id_;

    private: bool verbose_;

    private: boost::mutex mtx_;
    };


    class AeroControllerProto
    {
    public: AeroControllerProto(const std::string& _port, uint8_t _id);

    public: ~AeroControllerProto();

    /// @brief servo on command
    public: void servo_on();

    /// @brief servo off command
    public: void servo_off();

    /// @brief servo toggle command
    /// @param _d0 1: on, 0: off
    protected: void servo_command(int16_t _d0);

    /// @brief send Get_Pos command
    /// @param _stroke_vector stroke vector
    public: void get_position(std::vector<int16_t>& _stroke_vector);

    /// @brief send Get_Cur command
    /// @param _stroke_vector stroke vector
    public: void get_current(std::vector<int16_t>& _stroke_vector);

    /// @brief send Get_Tmp command
    /// @param _stroke_vector stroke vector
    public: void get_temperature(std::vector<int16_t>& _stroke_vector);

    /// @brief get data from buffer,
    ///   this does not call command, but only read from buffer
    /// @param _stroke_vector stroke vector
    public: void get_data(std::vector<int16_t>& _stroke_vector);

    public: std::vector<int16_t>& get_reference_stroke_vector();

    public: std::string get_stroke_joint_name(size_t _idx);

    public: int get_number_of_angle_joints();

    public: int32_t get_ordered_angle_id(std::string _name);

    /// @brief abstract of get commands
    /// @param _cmd command id
    /// @param _stroke_vector stroke vector
    protected: void get_command(uint8_t _cmd,
				std::vector<int16_t>& _stroke_vector);

    /// @brief set position command
    /// @param _stroke_vector stroke vector, MUST be DOF bytes
    /// @param _time time[ms]
    public: void set_position(std::vector<int16_t>& _stroke_vector,
			      uint16_t _time);

    /// @brief send Motor_Cur command
    /// @param _stroke_vector stroke vector
    public: void set_max_current(std::vector<int16_t>& _stroke_vector);

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

    public: void flush();

    /// @brief decode short(int16_t) from byte(uint8_t)
    protected: int16_t decode_short_(uint8_t* _raw);

    /// @brief ecnode short(int16_t) to byte(uint8_t)
    protected: void encode_short_(int16_t _value, uint8_t* _raw);

    /// @brief stoke_vector to raw command bytes
    protected: void stroke_to_raw_(std::vector<int16_t>& _stroke,
				   std::vector<uint8_t>& _raw);

    /// @brief raw command bytes to stoke_vector
    protected: void raw_to_stroke_(std::vector<uint8_t>& _raw,
				   std::vector<int16_t>& _stroke);

    protected: bool verbose_;

    protected: boost::mutex ctrl_mtx_;

    protected: SEED485Controller ser_;

    protected: std::vector<int16_t> stroke_vector_;

    protected: std::vector<int16_t> stroke_ref_vector_;

    protected: std::vector<int16_t> stroke_cur_vector_;

    protected: std::vector<AJointIndex> stroke_joint_indices_;

    protected:
      std::unordered_map<std::string, int32_t> angle_joint_indices_;
    };

  }
}

#endif
