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

using namespace boost::asio;

namespace {
const static uint8_t CMD_MOTOR_CUR = 0x20;
const static uint8_t CMD_MOTOR_ACC = 0x24;
const static uint8_t CMD_MOTOR_GAIN = 0x25;
const static uint8_t CMD_GET_POS = 0x42;
const static uint8_t CMD_GET_TMP = 0x43;
const static uint8_t CMD_GET_CUR = 0x45;
const static uint8_t CMD_MOTOR_SRV = 0x50;
const static uint8_t CMD_MOVE_INC = 0x67;
const static uint8_t CMD_MOVE_ABS = 0x68;
}

class AeroController {
 public:
  AeroController(io_service& ios, std::string& port);
  ~AeroController();

  void seed_485_send(std::vector<uint8_t>& send_data);
  void seed_485_read(std::vector<uint8_t>& read_data);
  void flush();

  void set_command(uint8_t id, uint8_t cmd, uint16_t time,
                   std::vector<int16_t>& values);
  void get_command(uint8_t id, uint8_t cmd);

  void servo_command(uint8_t id, int16_t d0);

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

};


#endif  // HRPSYS_AERO_BRIDGE_AERO_CONTROLLER_HPP_
