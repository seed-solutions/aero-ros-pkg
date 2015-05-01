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

  AJointIndex(size_t i, size_t sidx, size_t ridx) :
      id(i), stroke_index(sidx), raw_index(ridx) {
  }
  AJointIndex(const AJointIndex& aji) {
    id = aji.id;
    stroke_index = aji.stroke_index;
    raw_index = aji.raw_index;
  }
  AJointIndex& operator=(const AJointIndex& aji) {
    id = aji.id;
    stroke_index = aji.stroke_index;
    raw_index = aji.raw_index;
    return *this;
  }
};

class AeroController {
 public:
  AeroController(io_service& ios, std::string& port);
  ~AeroController();

  // basic commands
  void seed_485_send(std::vector<uint8_t>& send_data);
  void seed_485_read(std::vector<uint8_t>& read_data);
  void flush();

  void set_command_header(uint8_t id, uint8_t cmd, uint16_t time,
                          std::vector<uint8_t>& dat);
  void set_check_sum(std::vector<uint8_t>& dat);

  // servo
  void servo_command(int16_t d0);
  void servo_on();
  void servo_off();

  void set_position(std::vector<int16_t>& stroke_vector, uint16_t time);
  void get_position(std::vector<int16_t>& stroke_vector);

  bool verbose() {return verbose_;}
  void verbose(bool v) {verbose_ = v;}

 private:
  io_service& io_;
  serial_port ser_;
  bool verbose_;

  std::vector<int16_t> stroke_vector_;
  std::vector<int16_t> stroke_ref_vector_;
  std::vector<int16_t> stroke_cur_vector_;

  std::vector<AJointIndex> joint_indices_;

  int16_t decode_short_(uint8_t* raw);
  void encode_short_(int16_t value, uint8_t* raw);

  /// @brief stroke_vector (int16_t) to raw_vector(uint8_t)
  void stroke_to_raw_(std::vector<int16_t>& stroke, std::vector<uint8_t>& raw);

  /// @brief raw_vector(uint8_t) to stroke_vector (int16_t)
  void raw_to_stroke_(std::vector<uint8_t>& raw, std::vector<int16_t>& stroke);
};

}  // namespace


#endif  // HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_HPP_
