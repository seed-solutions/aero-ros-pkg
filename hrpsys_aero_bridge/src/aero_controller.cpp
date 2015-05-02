#include "hrpsys_aero_bridge/aero_controller.hpp"

using namespace boost::asio;

namespace aero_controller {

SEED485Controller::SEED485Controller(std::string& port, uint8_t id):
    ser_(io_), verbose_(true), id_(id) {
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

SEED485Controller::~SEED485Controller() {
  if (ser_.is_open()) {
    ser_.close();
  }
}


/// @brief send command
/// @param send_data data (uint8_t)
void SEED485Controller::send(std::vector<uint8_t>& send_data) {
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

/// @brief recv command
/// @param read_data data (uint8_t)
void SEED485Controller::read(std::vector<uint8_t>& read_data) {
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

/// @brief flush io buffer
void SEED485Controller::flush() {
  if (ser_.is_open()) {
    ::tcflush(ser_.lowest_layer().native_handle(), TCIOFLUSH);
  }
}

/// @brief create command header into data buffer
/// @param id id of bus
/// @param cmd command id
/// @param time time[ms]
/// @param data buffer, must be RAW_DATA_LENGTH(77) bytes.
void SEED485Controller::set_command_header(
    uint8_t cmd, uint16_t time, std::vector<uint8_t>& dat) {
  dat[0] = 0xFA;
  dat[1] = 0xAF;
  dat[2] = 0xF0 + id_;
  dat[3] = cmd;

  //  time (2 bytes)
  dat[4] = static_cast<uint8_t>(0xff & (time >> 8));
  dat[5] = static_cast<uint8_t>(0xff & time);
}

/// @brief calc checksum and put into last byte, It MUST be called LAST.
/// @param data buffer, MUST be RAW_DATA_LENGTH(77) bytes.
void SEED485Controller::set_check_sum(std::vector<uint8_t>& dat) {
	int32_t b_check_sum = 0;
  for (size_t i = 2; i < RAW_DATA_LENGTH - 1; i++) {
    b_check_sum += dat[i];
  }
  dat[RAW_DATA_LENGTH - 1] =
      ~(reinterpret_cast<uint8_t*>(&b_check_sum)[0]);
}

void SEED485Controller::send_command(
    uint8_t cmd, uint16_t time, std::vector<uint8_t>& send_data) {
  set_command_header(cmd, time, send_data);
  set_check_sum(send_data);
  send(send_data);
}



AeroController::AeroController(std::string& port_upper,
                               std::string& port_lower):
    ser_upper_(port_upper, ID_UPPER), ser_lower_(port_lower, ID_LOWER),
    verbose_(true) {
  // stroke_vector
  stroke_vector_.resize(46);
  stroke_ref_vector_.resize(46);
  stroke_cur_vector_.resize(46);

  // indices
  joint_indices_.clear();
  // neck
  joint_indices_.push_back(
      AJointIndex(1, STROKE_NECK_Y, RAW_NECK_Y));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_NECK_RIGHT, RAW_NECK_RIGHT));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_NECK_LEFT, RAW_NECK_LEFT));
  // rarm
  joint_indices_.push_back(
      AJointIndex(1, STROKE_RIGHT_SHOULDER_P, RAW_RIGHT_SHOULDER_P));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_RIGHT_SHOULDER_R, RAW_RIGHT_SHOULDER_R));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_RIGHT_ELBOW_Y, RAW_RIGHT_ELBOW_Y));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_RIGHT_ELBOW_P, RAW_RIGHT_ELBOW_P));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_RIGHT_WRIST_R, RAW_RIGHT_WRIST_R));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_RIGHT_WRIST_TOP, RAW_RIGHT_WRIST_TOP));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_RIGHT_WRIST_BOTTOM, RAW_RIGHT_WRIST_BOTTOM));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_RIGHT_HAND, RAW_RIGHT_HAND));
  // larm
  joint_indices_.push_back(
      AJointIndex(1, STROKE_LEFT_SHOULDER_P, RAW_LEFT_SHOULDER_P));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_LEFT_SHOULDER_R, RAW_LEFT_SHOULDER_R));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_LEFT_ELBOW_Y, RAW_LEFT_ELBOW_Y));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_LEFT_ELBOW_P, RAW_LEFT_ELBOW_P));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_LEFT_WRIST_R, RAW_LEFT_WRIST_R));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_LEFT_WRIST_TOP, RAW_LEFT_WRIST_TOP));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_LEFT_WRIST_BOTTOM, RAW_LEFT_WRIST_BOTTOM));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_LEFT_HAND, RAW_LEFT_HAND));
  // waist
  joint_indices_.push_back(
      AJointIndex(1, STROKE_WAIST_RIGHT, RAW_WAIST_RIGHT));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_WAIST_LEFT, RAW_WAIST_LEFT));
  joint_indices_.push_back(
      AJointIndex(1, STROKE_WAIST_P, RAW_WAIST_P));
  // frleg
  joint_indices_.push_back(
      AJointIndex(2, STROKE_FRONT_RIGHT_CROTCH_Y, RAW_FRONT_RIGHT_CROTCH_Y));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_FRONT_RIGHT_CROTCH_P, RAW_FRONT_RIGHT_CROTCH_P0));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_FRONT_RIGHT_KNEE_P, RAW_FRONT_RIGHT_KNEE_P0));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_FRONT_RIGHT_WHEEL, RAW_FRONT_RIGHT_WHEEL));
  // rrleg
  joint_indices_.push_back(
      AJointIndex(2, STROKE_REAR_RIGHT_CROTCH_Y, RAW_REAR_RIGHT_CROTCH_Y));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_REAR_RIGHT_CROTCH_P, RAW_REAR_RIGHT_CROTCH_P0));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_REAR_RIGHT_KNEE_P, RAW_REAR_RIGHT_KNEE_P0));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_REAR_RIGHT_WHEEL, RAW_REAR_RIGHT_WHEEL));
  // flleg
  joint_indices_.push_back(
      AJointIndex(2, STROKE_FRONT_LEFT_CROTCH_Y, RAW_FRONT_LEFT_CROTCH_Y));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_FRONT_LEFT_CROTCH_P, RAW_FRONT_LEFT_CROTCH_P0));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_FRONT_LEFT_KNEE_P, RAW_FRONT_LEFT_KNEE_P0));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_FRONT_LEFT_WHEEL, RAW_FRONT_LEFT_WHEEL));
  // rlleg
  joint_indices_.push_back(
      AJointIndex(2, STROKE_REAR_LEFT_CROTCH_Y, RAW_REAR_LEFT_CROTCH_Y));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_REAR_LEFT_CROTCH_P, RAW_REAR_LEFT_CROTCH_P0));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_REAR_LEFT_KNEE_P, RAW_REAR_LEFT_KNEE_P0));
  joint_indices_.push_back(
      AJointIndex(2, STROKE_REAR_LEFT_WHEEL, RAW_REAR_LEFT_WHEEL));

  // // debug
  // for (size_t i = 0; i < joint_indices_.size(); i++) {
  //   AJointIndex& aji = joint_indices_[i];
  //   std::cout << "joint[" << i << "]("
  //             << "id: " << aji.id << ", "
  //             << "stroke: " << aji.stroke_index << ", "
  //             << "raw: " << aji.raw_index << ")" << std::endl;
  // }
  // //
}

AeroController::~AeroController() {
}

void AeroController::flush() {
  ser_upper_.flush();
  ser_lower_.flush();
}

int16_t AeroController::decode_short_(uint8_t* raw) {
  int16_t value;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&value);
  bvalue[0] = raw[1];
  bvalue[1] = raw[0];
  return value;
}
void AeroController::encode_short_(int16_t value, uint8_t* raw) {
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&value);
  raw[0] = bvalue[1];
  raw[1] = bvalue[0];
}

void AeroController::stroke_to_raw_(std::vector<int16_t>& stroke,
                                    std::vector<uint8_t>& raw_upper,
                                    std::vector<uint8_t>& raw_lower) {
  for (size_t i = 0; i < joint_indices_.size(); i++) {
    AJointIndex& aji = joint_indices_[i];
    if (aji.id == 1) {
      encode_short_(stroke[aji.stroke_index],
                    &raw_upper[RAW_HEADER_OFFSET + aji.raw_index * 2]);
    }
  }
}
void AeroController::raw_to_stroke_(std::vector<uint8_t>& raw_upper,
                                    std::vector<uint8_t>& raw_lower,
                                    std::vector<int16_t>& stroke) {
  for (size_t i = 0; i < joint_indices_.size(); i++) {
    AJointIndex& aji = joint_indices_[i];
    if (aji.id == 1) {
      stroke[aji.stroke_index] =
          decode_short_(&raw_upper[RAW_HEADER_OFFSET + aji.raw_index * 2]);
    }
  }
}


/// @brief servo toggle command
/// @param d0 1: on, 0: off
void AeroController::servo_command(int16_t d0) {
  std::vector<int16_t> stroke_vector;
  stroke_vector.resize(AERO_DOF);
  for (size_t i = 0; i < stroke_vector.size(); i++) {
    stroke_vector[i] = d0;
  }

  std::vector<uint8_t> dat_upper, dat_lower;
  dat_upper.resize(RAW_DATA_LENGTH);
  dat_lower.resize(RAW_DATA_LENGTH);

  // ser_upper_.set_command_header(CMD_MOTOR_SRV, 0, dat_upper);
  // ser_lower_.set_command_header(CMD_MOTOR_SRV, 0, dat_lower);

  stroke_to_raw_(stroke_vector, dat_upper, dat_lower);

  // ser_upper_.set_check_sum(dat_upper);
  // ser_lower_.set_check_sum(dat_lower);

  // ser_upper_.send(dat_upper);
  // ser_lower_.send(dat_lower);

  ser_upper_.send_command(CMD_MOTOR_SRV, 0, dat_upper);
  ser_lower_.send_command(CMD_MOTOR_SRV, 0, dat_lower);
}

/// @brief servo on command
void AeroController::servo_on() {
  servo_command(1);
}

/// @brief servo off command
void AeroController::servo_off() {
  servo_command(0);
}

/// @brief set position command
/// @param stroke_vector stroke vector, MUST be AERO_DOF(46) bytes
/// @param time time[ms]
void AeroController::set_position(std::vector<int16_t>& stroke_vector,
                                  uint16_t time) {
  std::vector<uint8_t> dat_upper, dat_lower;
  dat_upper.resize(RAW_DATA_LENGTH);
  dat_lower.resize(RAW_DATA_LENGTH);

  // ser_upper_.set_command_header(CMD_MOVE_ABS, time, dat_upper);
  // ser_lower_.set_command_header(CMD_MOVE_ABS, time, dat_lower);

  stroke_to_raw_(stroke_vector, dat_upper, dat_lower);

  // ser_upper_.set_check_sum(dat_upper);
  // ser_lower_.set_check_sum(dat_lower);

  // ser_upper_.send(dat_upper);
  // ser_lower_.send(dat_lower);

  ser_upper_.send_command(CMD_MOVE_ABS, time, dat_upper);
  ser_lower_.send_command(CMD_MOVE_ABS, time, dat_lower);
}

/// @brief get position, this does not call command, but only read from buffer
/// @param stroke_vector stroke vector
void AeroController::get_position(std::vector<int16_t>& stroke_vector) {
  std::vector<uint8_t> dat_upper, dat_lower;
  dat_upper.resize(RAW_DATA_LENGTH);
  dat_lower.resize(RAW_DATA_LENGTH);

  ser_upper_.read(dat_upper);
  ser_lower_.read(dat_lower);

  stroke_vector.resize(AERO_DOF);
  raw_to_stroke_(dat_upper, dat_lower, stroke_vector);
}

}  // namespace
