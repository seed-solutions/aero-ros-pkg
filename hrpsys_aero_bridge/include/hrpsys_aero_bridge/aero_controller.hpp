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
#include <boost/thread.hpp>

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
  SEED485Controller(const std::string& port, uint8_t id);
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
  boost::mutex mtx_;
};

//////////////////////////////
/// Proto type of controller
class AeroControllerProto {
 public:
  AeroControllerProto(const std::string& port, uint8_t id);
  ~AeroControllerProto();

  // servo
  void servo_command(int16_t d0);
  void servo_on();
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

  void flush() {
    boost::mutex::scoped_lock(ctrl_mtx_);
    ser_.flush();
  }

  bool verbose() {return verbose_;}
  void verbose(bool v) {verbose_ = v;}

  int32_t get_stroke_index_from_joint_name(std::string& name);

  std::vector<int16_t>& get_reference_stroke_vector() {
    return stroke_ref_vector_;
  }

  std::string get_joint_name(size_t idx) {
    return joint_indices_[idx].joint_name;
  }

 protected:
  bool verbose_;
  boost::mutex ctrl_mtx_;

  SEED485Controller ser_;

  std::vector<int16_t> stroke_vector_;
  std::vector<int16_t> stroke_ref_vector_;
  std::vector<int16_t> stroke_cur_vector_;

  std::vector<AJointIndex> joint_indices_;

  int16_t decode_short_(uint8_t* raw);
  void encode_short_(int16_t value, uint8_t* raw);

  /// @brief stroke_vector (int16_t) to raw_vector(uint8_t)
  void stroke_to_raw_(std::vector<int16_t>& stroke,
                      std::vector<uint8_t>& raw);

  /// @brief raw_vector(uint8_t) to stroke_vector (int16_t)
  void raw_to_stroke_(std::vector<uint8_t>& raw,
                      std::vector<int16_t>& stroke);
};


////////////////////////////////
/// Upper body
class AeroUpperController : public AeroControllerProto {
 public:
  AeroUpperController(const std::string& port);
  ~AeroUpperController();
};

/// Lower body, including wheels
class AeroLowerController : public AeroControllerProto {
 public:
  AeroLowerController(const std::string& port);
  ~AeroLowerController();

  // servo: spliting wheels and joints
  void servo_command(int16_t d0, int16_t d1);
  void servo_on();
  void wheel_on();
  void servo_off();

  // wheel command
  void set_wheel_velocity(std::vector<int16_t>& wheel_vector,
                          uint16_t time);
  int32_t get_wheel_index_from_wheel_name(std::string& name);
  std::vector<int16_t>& get_reference_wheel_vector() {
    return wheel_ref_vector_;
  }
  std::string get_wheel_name(size_t idx) {
    return wheel_indices_[idx].joint_name;
  }

 protected:
  std::vector<int16_t> wheel_vector_;
  std::vector<int16_t> wheel_ref_vector_;
  std::vector<int16_t> wheel_cur_vector_;

  std::vector<AJointIndex> wheel_indices_;

  bool wheel_servo_;
};

}  // namespace


#endif  // HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_HPP_
