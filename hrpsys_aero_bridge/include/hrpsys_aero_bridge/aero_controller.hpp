#ifndef HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_HPP_
#define HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_HPP_

#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
#include <string>
#include <stdint.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/algorithm/string.hpp>

#include "hrpsys_aero_bridge/constants.hpp"

using namespace boost::asio;

namespace aero_controller {

class AJointIndex {
 public:
  size_t id;
  size_t stroke_index;
  size_t raw_index;
  std::string joint_name;

  AJointIndex(size_t i, size_t sidx, size_t ridx, std::string name) :
      id(i), stroke_index(sidx), raw_index(ridx), joint_name(name) {
  }
  AJointIndex(const AJointIndex& aji) {
    id = aji.id;
    stroke_index = aji.stroke_index;
    raw_index = aji.raw_index;
    joint_name = aji.joint_name;
  }
  AJointIndex& operator=(const AJointIndex& aji) {
    id = aji.id;
    stroke_index = aji.stroke_index;
    raw_index = aji.raw_index;
    joint_name = aji.joint_name;
    return *this;
  }
};

class SEED485Controller {
 public:
  SEED485Controller(std::string& port, uint8_t id);
  ~SEED485Controller();

  // basic commands
  void send(std::vector<uint8_t>& send_data);
  void read(std::vector<uint8_t>& read_data);
  void flush();

  // command operation
  void set_command_header(uint8_t cmd, uint16_t time,
                          std::vector<uint8_t>& dat);
  void set_check_sum(std::vector<uint8_t>& dat);
  void send_command(uint8_t cmd, uint16_t time,
                    std::vector<uint8_t>& send_data);

  bool verbose() {return verbose_;}
  void verbose(bool v) {verbose_ = v;}

 private:
  io_service io_;
  serial_port ser_;
  uint8_t id_;
  bool verbose_;
};


class AeroController {
 public:
  AeroController(std::string& port_upper, std::string& port_lower);
  ~AeroController();

  // flush all controllers
  void flush();

  // servo
  void servo_command(int16_t d0, int16_t d1);
  void servo_on();
  void wheel_on();
  void servo_off();

  // postion
  void set_position(std::vector<int16_t>& stroke_vector, uint16_t time);

  // get data
  void get_data(std::vector<int16_t>& stroke_vector);

  // set commands
  void set_command(uint8_t cmd, std::vector<int16_t>& stroke_vector);
  void set_max_current(std::vector<int16_t>& stroke_vector);
  void set_accel_rate(std::vector<int16_t>& stroke_vector);
  void set_motor_gain(std::vector<int16_t>& stroke_vector);

  // get commands
  void get_command(uint8_t cmd, std::vector<int16_t>& stroke_vector);
  void get_position(std::vector<int16_t>& stroke_vector);
  void get_current(std::vector<int16_t>& stroke_vector);
  void get_temperature(std::vector<int16_t>& stroke_vector);

  bool verbose() {return verbose_;}
  void verbose(bool v) {verbose_ = v;}

 private:
  bool verbose_;

  SEED485Controller ser_upper_;
  SEED485Controller ser_lower_;

  std::vector<int16_t> stroke_vector_;
  std::vector<int16_t> stroke_ref_vector_;
  std::vector<int16_t> stroke_cur_vector_;

  std::vector<AJointIndex> joint_indices_;

  int16_t decode_short_(uint8_t* raw);
  void encode_short_(int16_t value, uint8_t* raw);

  /// @brief stroke_vector (int16_t) to raw_vector(uint8_t)
  void stroke_to_raw_(std::vector<int16_t>& stroke,
                      std::vector<uint8_t>& raw_upper,
                      std::vector<uint8_t>& raw_lower);

  /// @brief raw_vector(uint8_t) to stroke_vector (int16_t)
  void raw_to_stroke_(std::vector<uint8_t>& raw_upper,
                      std::vector<uint8_t>& raw_lower,
                      std::vector<int16_t>& stroke);
};

}  // namespace


#endif  // HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_HPP_
