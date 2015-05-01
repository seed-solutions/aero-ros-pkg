#include "hrpsys_aero_bridge/aero_controller.hpp"

using namespace aero_controller;

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

  aero.servo_on();

  // flush i/o
  usleep(5000 * 1000);
  aero.flush();

  std::vector<int16_t> stroke_vector;
  std::vector<int16_t> stroke_vector_current;
  stroke_vector.resize(AERO_DOF);
  stroke_vector_current.resize(AERO_DOF);

  aero.set_position(stroke_vector, 2000);
  usleep(5000 * 1000);

  for (size_t j = 0; j < 100; j++) {
    int16_t y = static_cast<int16_t>(a * sin(omega * i));
    stroke_vector.clear();
    stroke_vector.resize(AERO_DOF);
    for (size_t k = 0; k < AERO_DOF; k++) {
      if (k == STROKE_NECK_Y ||
          k == STROKE_RIGHT_ELBOW_Y ||
          k == STROKE_WAIST_P ||
          k == STROKE_LEFT_ELBOW_Y ||
          k == STROKE_LEFT_WRIST_R) {
        stroke_vector[k] = y * 5;
      } else if (k == STROKE_NECK_RIGHT ||
                 k == STROKE_NECK_LEFT ||
                 k == STROKE_RIGHT_WRIST_TOP ||
                 k == STROKE_RIGHT_WRIST_BOTTOM) {
        stroke_vector[k] = y / 2;
      } else if (k == STROKE_RIGHT_WRIST_R) {
        stroke_vector[k] = y * 10;
      } else {
        stroke_vector[k] = y;
      }
    }

    aero.set_position(stroke_vector, move_time);
    aero.get_position(stroke_vector_current);

    std::cout << "send:";
    for (size_t vi = 0; vi < stroke_vector.size(); vi++) {
      std::cout << std::dec << " " << stroke_vector[vi];
    }
    std::cout << std::endl;

    std::cout << "recv:";
    for (size_t vi = 0; vi < stroke_vector_current.size(); vi++) {
      std::cout << std::dec << " " << stroke_vector_current[vi];
    }
    std::cout << std::endl;

    usleep(move_time * 1000);
    i += move_time * 0.001;
  }

  return 0;
}
