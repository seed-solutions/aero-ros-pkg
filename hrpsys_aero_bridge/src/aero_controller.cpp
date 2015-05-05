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
    boost::mutex::scoped_lock lock(mtx_);
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
  read_data.resize(77);
  if (ser_.is_open()) {
    boost::mutex::scoped_lock lock(mtx_);
    ser_.read_some(buffer(read_data, 77));
  }
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
  stroke_vector_.resize(AERO_DOF);
  stroke_ref_vector_.resize(AERO_DOF);
  stroke_cur_vector_.resize(AERO_DOF);

  // indices
  joint_indices_.clear();
  // neck
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_NECK_Y, RAW_NECK_Y,
                  std::string("neck_yaw_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_NECK_RIGHT, RAW_NECK_RIGHT,
                  std::string("neck_right_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_NECK_LEFT, RAW_NECK_LEFT,
                  std::string("neck_left_joint")));
  // rarm
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_RIGHT_SHOULDER_P, RAW_RIGHT_SHOULDER_P,
                  std::string("r_shoulder_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_RIGHT_SHOULDER_R, RAW_RIGHT_SHOULDER_R,
                  std::string("r_shoulder_roll_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_RIGHT_ELBOW_Y, RAW_RIGHT_ELBOW_Y,
                  std::string("r_elbow_yaw_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_RIGHT_ELBOW_P, RAW_RIGHT_ELBOW_P,
                  std::string("r_elbow_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_RIGHT_WRIST_R, RAW_RIGHT_WRIST_R,
                  std::string("r_wrist_roll_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_RIGHT_WRIST_TOP, RAW_RIGHT_WRIST_TOP,
                  std::string("r_wrist_top_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_RIGHT_WRIST_BOTTOM, RAW_RIGHT_WRIST_BOTTOM,
                  std::string("r_wrist_bottom_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_RIGHT_HAND, RAW_RIGHT_HAND,
                  std::string("r_hand_joint")));
  // larm
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_LEFT_SHOULDER_P, RAW_LEFT_SHOULDER_P,
                  std::string("l_shoulder_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_LEFT_SHOULDER_R, RAW_LEFT_SHOULDER_R,
                  std::string("l_shoulder_roll_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_LEFT_ELBOW_Y, RAW_LEFT_ELBOW_Y,
                  std::string("l_elbow_yaw_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_LEFT_ELBOW_P, RAW_LEFT_ELBOW_P,
                  std::string("l_elbow_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_LEFT_WRIST_R, RAW_LEFT_WRIST_R,
                  std::string("l_wrist_roll_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_LEFT_WRIST_TOP, RAW_LEFT_WRIST_TOP,
                  std::string("l_wrist_top_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_LEFT_WRIST_BOTTOM, RAW_LEFT_WRIST_BOTTOM,
                  std::string("l_wrist_bottom_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_LEFT_HAND, RAW_LEFT_HAND,
                  std::string("l_hand_joint")));
  // waist
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_WAIST_RIGHT, RAW_WAIST_RIGHT,
                  std::string("waist_right_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_WAIST_LEFT, RAW_WAIST_LEFT,
                  std::string("waist_left_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_UPPER, STROKE_WAIST_P, RAW_WAIST_P,
                  std::string("waist_pitch_joint")));
  // frleg
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_FRONT_RIGHT_CROTCH_Y, RAW_FRONT_RIGHT_CROTCH_Y,
                  std::string("f_r_crotch_yaw_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_FRONT_RIGHT_CROTCH_P, RAW_FRONT_RIGHT_CROTCH_P1,
                  std::string("f_r_crotch_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_FRONT_RIGHT_KNEE_P, RAW_FRONT_RIGHT_KNEE_P1,
                  std::string("f_r_knee_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER, STROKE_FRONT_RIGHT_WHEEL, RAW_FRONT_RIGHT_WHEEL,
                  std::string("f_r_wheel_joint")));
  // rrleg
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_REAR_RIGHT_CROTCH_Y, RAW_REAR_RIGHT_CROTCH_Y,
                  std::string("r_r_crotch_yaw_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_REAR_RIGHT_CROTCH_P, RAW_REAR_RIGHT_CROTCH_P1,
                  std::string("r_r_crotch_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_REAR_RIGHT_KNEE_P, RAW_REAR_RIGHT_KNEE_P1,
                  std::string("r_r_knee_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER, STROKE_REAR_RIGHT_WHEEL, RAW_REAR_RIGHT_WHEEL,
                  std::string("r_r_wheel_joint")));
  // flleg
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_FRONT_LEFT_CROTCH_Y, RAW_FRONT_LEFT_CROTCH_Y,
                  std::string("f_l_crotch_yaw_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_FRONT_LEFT_CROTCH_P, RAW_FRONT_LEFT_CROTCH_P1,
                  std::string("f_l_crotch_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_FRONT_LEFT_KNEE_P, RAW_FRONT_LEFT_KNEE_P1,
                  std::string("f_l_knee_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER, STROKE_FRONT_LEFT_WHEEL, RAW_FRONT_LEFT_WHEEL,
                  std::string("f_l_wheel_joint")));
  // rlleg
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_REAR_LEFT_CROTCH_Y, RAW_REAR_LEFT_CROTCH_Y,
                  std::string("r_l_crotch_yaw_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_REAR_LEFT_CROTCH_P, RAW_REAR_LEFT_CROTCH_P1,
                  std::string("r_l_crotch_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER,
                  STROKE_REAR_LEFT_KNEE_P, RAW_REAR_LEFT_KNEE_P1,
                  std::string("r_l_knee_pitch_joint")));
  joint_indices_.push_back(
      AJointIndex(ID_LOWER, STROKE_REAR_LEFT_WHEEL, RAW_REAR_LEFT_WHEEL,
                  std::string("r_l_wheel_joint")));

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

/// @brief flush buffer
void AeroController::flush() {
  ser_upper_.flush();
  ser_lower_.flush();
}

/// @brief decode short(int16_t) from byte(uint8_t)
int16_t AeroController::decode_short_(uint8_t* raw) {
  int16_t value;
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&value);
  bvalue[0] = raw[1];
  bvalue[1] = raw[0];
  return value;
}
/// @brief ecnode short(int16_t) to byte(uint8_t)
void AeroController::encode_short_(int16_t value, uint8_t* raw) {
  uint8_t* bvalue = reinterpret_cast<uint8_t*>(&value);
  raw[0] = bvalue[1];
  raw[1] = bvalue[0];
}

/// @brief stoke_vector to raw command bytes
void AeroController::stroke_to_raw_(std::vector<int16_t>& stroke,
                                    std::vector<uint8_t>& raw_upper,
                                    std::vector<uint8_t>& raw_lower) {
  for (size_t i = 0; i < joint_indices_.size(); i++) {
    AJointIndex& aji = joint_indices_[i];
    if (aji.id == ID_UPPER) {
      encode_short_(stroke[aji.stroke_index],
                    &raw_upper[RAW_HEADER_OFFSET + aji.raw_index * 2]);
    } else if (aji.id == ID_LOWER) {
      encode_short_(stroke[aji.stroke_index],
                    &raw_lower[RAW_HEADER_OFFSET + aji.raw_index * 2]);
    }
  }
}
/// @brief raw command bytes to stoke_vector
void AeroController::raw_to_stroke_(std::vector<uint8_t>& raw_upper,
                                    std::vector<uint8_t>& raw_lower,
                                    std::vector<int16_t>& stroke) {
  for (size_t i = 0; i < joint_indices_.size(); i++) {
    AJointIndex& aji = joint_indices_[i];
    if (aji.id == ID_UPPER) {
      stroke[aji.stroke_index] =
          decode_short_(&raw_upper[RAW_HEADER_OFFSET + aji.raw_index * 2]);
    } else if (aji.id == ID_LOWER) {
      stroke[aji.stroke_index] =
          decode_short_(&raw_lower[RAW_HEADER_OFFSET + aji.raw_index * 2]);
    }
  }
}


/// @brief servo toggle command
/// @param d0 1: on, 0: off
/// @param d1 wheel servo, 1: on, 0: off
void AeroController::servo_command(int16_t d0, int16_t d1) {
  std::vector<int16_t> stroke_vector;
  stroke_vector.resize(AERO_DOF);
  for (size_t i = 0; i < stroke_vector.size(); i++) {
    stroke_vector[i] = d0;
    if (i == STROKE_FRONT_RIGHT_WHEEL ||
        i == STROKE_FRONT_LEFT_WHEEL ||
        i == STROKE_REAR_RIGHT_WHEEL ||
        i == STROKE_REAR_LEFT_WHEEL) {
      stroke_vector[i] = d1;
    }
  }

  std::vector<uint8_t> dat_upper, dat_lower;
  dat_upper.resize(RAW_DATA_LENGTH);
  dat_lower.resize(RAW_DATA_LENGTH);

  stroke_to_raw_(stroke_vector, dat_upper, dat_lower);
  // lower body: slave motors must be toggled by servo command?
  encode_short_(d0,
                &dat_lower[RAW_HEADER_OFFSET + RAW_FRONT_RIGHT_CROTCH_P0 * 2]);
  encode_short_(d0,
                &dat_lower[RAW_HEADER_OFFSET + RAW_FRONT_RIGHT_KNEE_P0 * 2]);
  encode_short_(d0,
                &dat_lower[RAW_HEADER_OFFSET + RAW_REAR_RIGHT_CROTCH_P0 * 2]);
  encode_short_(d0,
                &dat_lower[RAW_HEADER_OFFSET + RAW_REAR_RIGHT_KNEE_P0 * 2]);
  encode_short_(d0,
                &dat_lower[RAW_HEADER_OFFSET + RAW_FRONT_LEFT_CROTCH_P0 * 2]);
  encode_short_(d0,
                &dat_lower[RAW_HEADER_OFFSET + RAW_FRONT_LEFT_KNEE_P0 * 2]);
  encode_short_(d0,
                &dat_lower[RAW_HEADER_OFFSET + RAW_REAR_LEFT_CROTCH_P0 * 2]);
  encode_short_(d0,
                &dat_lower[RAW_HEADER_OFFSET + RAW_REAR_LEFT_KNEE_P0 * 2]);
  //

  ser_upper_.send_command(CMD_MOTOR_SRV, 0, dat_upper);
  ser_lower_.send_command(CMD_MOTOR_SRV, 0, dat_lower);
}

/// @brief servo on command, wheels will servo off
void AeroController::servo_on() {
  servo_command(1, 0);
}

/// @brief servo on command including wheel
///   if you want to servo off only wheel, call servo_on()
void AeroController::wheel_on() {
  servo_command(1, 1);
}

/// @brief servo off command
void AeroController::servo_off() {
  servo_command(0, 0);
}

/// @brief set position command
/// @param stroke_vector stroke vector, MUST be AERO_DOF(46) bytes
/// @param time time[ms]
void AeroController::set_position(std::vector<int16_t>& stroke_vector,
                                  uint16_t time) {
  std::vector<uint8_t> dat_upper, dat_lower;
  dat_upper.resize(RAW_DATA_LENGTH);
  dat_lower.resize(RAW_DATA_LENGTH);

  stroke_to_raw_(stroke_vector, dat_upper, dat_lower);
  stroke_ref_vector_.assign(stroke_vector.begin(), stroke_vector.end());

  ser_upper_.send_command(CMD_MOVE_ABS, time, dat_upper);
  ser_lower_.send_command(CMD_MOVE_ABS, time, dat_lower);
}

/// @brief get data from buffer,
///   this does not call command, but only read from buffer
/// @param stroke_vector stroke vector
void AeroController::get_data(std::vector<int16_t>& stroke_vector) {
  std::vector<uint8_t> dat_upper, dat_lower;
  dat_upper.resize(RAW_DATA_LENGTH);
  dat_lower.resize(RAW_DATA_LENGTH);

  ser_upper_.read(dat_upper);
  ser_lower_.read(dat_lower);

  stroke_vector.resize(AERO_DOF);
  raw_to_stroke_(dat_upper, dat_lower, stroke_vector);
}


/// @brief abstract of set commands
/// @param cmd command id
/// @param stroke_vector stroke vector
void AeroController::set_command(uint8_t cmd,
                                 std::vector<int16_t>& stroke_vector) {
  std::vector<uint8_t> dat_upper, dat_lower;
  dat_upper.resize(RAW_DATA_LENGTH);
  dat_lower.resize(RAW_DATA_LENGTH);
  stroke_to_raw_(stroke_vector, dat_upper, dat_lower);
  ser_upper_.send_command(cmd, 0, dat_upper);
  ser_lower_.send_command(cmd, 0, dat_lower);
}
/// @brief send Motor_Cur command
/// @param stroke_vector stroke vector
void AeroController::set_max_current(std::vector<int16_t>& stroke_vector) {
  set_command(CMD_MOTOR_CUR, stroke_vector);
}
/// @brief send Motor_Acc command
/// @param stroke_vector stroke vector
void AeroController::set_accel_rate(std::vector<int16_t>& stroke_vector) {
  set_command(CMD_MOTOR_ACC, stroke_vector);
}
/// @brief send Motor_Gain command
/// @param stroke_vector stroke vector
void AeroController::set_motor_gain(std::vector<int16_t>& stroke_vector) {
  set_command(CMD_MOTOR_GAIN, stroke_vector);
}


/// @brief abstract of get commands
/// @param cmd command id
/// @param stroke_vector stroke vector
void AeroController::get_command(uint8_t cmd,
                                 std::vector<int16_t>& stroke_vector) {
  std::vector<uint8_t> dat_upper, dat_lower;
  dat_upper.resize(RAW_DATA_LENGTH);
  dat_lower.resize(RAW_DATA_LENGTH);
  ser_upper_.send_command(cmd, 0, dat_upper);
  ser_lower_.send_command(cmd, 0, dat_lower);
  get_data(stroke_vector);
}

/// @brief send Get_Pos command
/// @param stroke_vector stroke vector
void AeroController::get_position(std::vector<int16_t>& stroke_vector) {
  get_command(CMD_GET_POS, stroke_vector);
}
/// @brief send Get_Cur command
/// @param stroke_vector stroke vector
void AeroController::get_current(std::vector<int16_t>& stroke_vector) {
  get_command(CMD_GET_CUR, stroke_vector);
}
/// @brief send Get_Tmp command
/// @param stroke_vector stroke vector
void AeroController::get_temperature(std::vector<int16_t>& stroke_vector) {
  get_command(CMD_GET_TMP, stroke_vector);
}


/// @brief return stroke index from joint name
/// @param name joint name
/// @return index in stroke vector, if not found, return -1
int32_t AeroController::get_stroke_index_from_joint_name(std::string& name) {
  for (size_t i = 0; i < joint_indices_.size(); i++) {
    if (joint_indices_[i].joint_name == name) {
      return static_cast<int32_t>(joint_indices_[i].stroke_index);
    }
  }
  return -1;
}


}  // namespace
