#include "hrpsys_aero_bridge/aero_controller.hpp"

using namespace boost::asio;

AeroController::AeroController(io_service& ios, std::string& port):
    io_(ios), ser_(io_), verbose_(true) {
  if (port != "") {
    boost::system::error_code err;

    ser_.open(port, err);
    if (err) {
      std::cerr << "could not open " << port << std::endl;
    } else {
      try {
        // ser_.set_option(serial_port_base::baud_rate(1382400));
        // ser_.set_option(serial_port_base::baud_rate(115200));
        struct termios tio;
        ::tcgetattr(ser_.lowest_layer().native_handle(), &tio);
        ::cfsetospeed(&tio, 1382400);
        ::cfsetispeed(&tio, 1382400);
        ::tcsetattr(ser_.lowest_layer().native_handle(), TCSANOW, &tio);
      } catch (std::exception& e) {
        std::cerr << "baudrate: " << e.what() << std::endl;
      }

      try {
        ser_.set_option(serial_port_base::character_size(
            serial_port_base::character_size(8)));
        ser_.set_option(serial_port_base::flow_control(
            serial_port_base::flow_control::none));
        ser_.set_option(serial_port_base::parity(
            serial_port_base::parity::none));
        ser_.set_option(serial_port_base::stop_bits(
            serial_port_base::stop_bits::one));
      } catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
      }
    }
  }
}

AeroController::~AeroController() {
  if (ser_.is_open()) {
    ser_.close();
  }
}


void AeroController::seed_485_send(std::vector<uint8_t>& send_data) {
  if (ser_.is_open()) {
    ser_.write_some(buffer(send_data));
  }
  if (verbose_) {
    std::cout << "send: ";
    for (size_t i = 0; i < send_data.size(); i++) {
      std::cout << std::uppercase << std::hex
                << std::setw(2) << std::setfill('0')
                << static_cast<int32_t>(send_data[i]);
    }
    std::cout << std::endl;
  }
}

void AeroController::seed_485_read(std::vector<uint8_t>& read_data) {
  if (ser_.is_open()) {
    read_data.resize(77);
    ser_.read_some(buffer(read_data, 77));
    if (verbose_) {
      std::cout << "recv: ";
      for (size_t i = 0; i < read_data.size(); i++) {
        std::cout << std::setw(2)
                  << std::uppercase << std::hex
                  << std::setw(2) << std::setfill('0')
                  << static_cast<int32_t>(read_data[i]);
      }
      std::cout << std::endl;
    }
  }
}

void AeroController::flush() {
  ::tcflush(ser_.lowest_layer().native_handle(), TCIOFLUSH);
}


void AeroController::set_command(uint8_t id, uint8_t cmd, uint16_t time,
                                 std::vector<int16_t>& values) {
	int32_t b_check_sum = 0;
	size_t b_length = 77;  // number of data +1

  std::vector<uint8_t> dat;
  dat.resize(b_length);
  dat[0] = 0xFA;
  dat[1] = 0xAF;
  dat[2] = 0xF0 + id;
  dat[3] = cmd;

  //  time (2 bytes)
  dat[4] = static_cast<uint8_t>(0xff & (time >> 8));
  dat[5] = static_cast<uint8_t>(0xff & time);

  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&values[0]);
  for (size_t i = 0; i < values.size(); i++) {
    dat[6 + i * 2] = bvalue[i * 2 + 1];
    dat[7 + i * 2] = bvalue[i * 2];
  }

  // checksum
  for (size_t i = 2; i < b_length - 1; i++) {
    b_check_sum += dat[i];
  }
  dat[b_length - 1] =
      ~(reinterpret_cast<uint8_t*>(&b_check_sum)[0]);

  seed_485_send(dat);
}
void AeroController::get_command(uint8_t id, uint8_t cmd) {

}
void AeroController::servo_command(uint8_t id, int16_t d0) {
  std::vector<int16_t> values;
  values.resize(35);
  for (size_t i = 0; i < values.size(); i++) {
    values[i] = d0;
  }
  set_command(id, CMD_MOTOR_SRV, 0, values);
}

void AeroController::servo_on() {
  servo_command(1, 1);
}
void AeroController::servo_off() {
  servo_command(1, 0);
}
void AeroController::set_position(std::vector<int16_t>& stroke_vector,
                                  uint16_t time) {
  set_command(1, CMD_MOVE_ABS, time, stroke_vector);
}
void AeroController::get_position(std::vector<int16_t>& stroke_vector) {
  std::vector<uint8_t> dat;
  dat.resize(77);
  seed_485_read(dat);

  // convert uint8_t to int16_t
  stroke_vector.resize(35);
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&stroke_vector[0]);
  for (size_t i = 0; i < stroke_vector.size(); i++) {
    bvalue[i * 2 + 1] = dat [6 + i * 2];
    bvalue[i * 2] = dat [7 + i * 2];
  }
}
