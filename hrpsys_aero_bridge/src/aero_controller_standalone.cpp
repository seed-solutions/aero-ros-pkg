#include "hrpsys_aero_bridge/aero_controller.hpp"

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
