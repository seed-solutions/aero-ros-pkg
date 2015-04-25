#include "hrpsys_aero_bridge/aero_controller.hpp"

using namespace boost::asio;

AeroController::AeroController(io_service& ios, std::string& port):
    io_(ios), ser_(io_) {
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
  std::cout << "send: ";
  for (size_t i = 0; i < send_data.size(); i++) {
    std::cout << std::uppercase << std::hex
              << std::setw(2) << std::setfill('0')
              << static_cast<int32_t>(send_data[i]);
  }
  std::cout << std::endl;
}

void AeroController::seed_485_read(std::vector<uint8_t>& read_data) {
  if (ser_.is_open()) {
    read_data.resize(77);
    ser_.read_some(buffer(read_data, 77));
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

void AeroController::flush() {
  ::tcflush(ser_.lowest_layer().native_handle(), TCIOFLUSH);
}



int main(int argc, char** argv) {
  std::string port("/dev/ttyUSB0");
  // std::string port("");
  io_service ios;

  AeroController aero(ios, port);

  double i = 0;
  double f = 0.5;
  double omega = 2.0 * M_PI * f;
  int32_t a = 100;
  int32_t move_time = 20;

  aero.servo_command(1, 1);

  // flush i/o
  usleep(5000 * 1000);
  aero.flush();

  std::vector<int16_t> values;
  std::vector<uint8_t> read_data;
  values.resize(35);

  aero.set_command(1, CMD_MOVE_ABS, 2000, values);
  //aero.seed_485_read(read_data);

  usleep(5000 * 1000);



  for (size_t j = 0; j < 100; j++) {
    int16_t y = static_cast<int16_t>(a * sin(omega * i));
    values.clear();
    values.resize(35);
    for (size_t k = 0; k < 35; k++) {
      switch (k) {
        case 0:
        case 5:
        case 16:
        case 21:
        case 23:
          values[k] = y * 5;
          break;
        case 1:
        case 2:
        case 8:
        case 9:
          values[k] = y / 2;
          break;
        case 7:
          values[k] = y * 10;
          break;
        default:
          values[k] = y;
      }
    }

    aero.set_command(1, CMD_MOVE_ABS, move_time, values);
    aero.seed_485_read(read_data);

    usleep(move_time * 1000);
    i += move_time * 0.001;
  }

  return 0;
}
