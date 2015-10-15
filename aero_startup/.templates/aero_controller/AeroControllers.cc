#include "aero_controller/AeroControllers.hh"

using namespace aero;
using namespace controller;

//////////////////////////////////////////////////
AeroUpperController::AeroUpperController(const std::string& _port) :
    AeroControllerProto(_port, ID_UPPER)
{
  stroke_vector_.resize(AERO_DOF_UPPER);
  stroke_ref_vector_.resize(AERO_DOF_UPPER);
  stroke_cur_vector_.resize(AERO_DOF_UPPER);

  stroke_joint_indices_.clear();
  stroke_joint_indices_.reserve(AERO_DOF_UPPER);

  // adding code
}

//////////////////////////////////////////////////
AeroUpperController::~AeroUpperController()
{
}

//////////////////////////////////////////////////
AeroLowerController::AeroLowerController(const std::string& _port) :
    AeroControllerProto(_port, ID_LOWER)
{
  stroke_vector_.resize(AERO_DOF_LOWER);
  stroke_ref_vector_.resize(AERO_DOF_LOWER);
  stroke_cur_vector_.resize(AERO_DOF_LOWER);

  wheel_vector_.resize(AERO_DOF_WHEEL);
  wheel_ref_vector_.resize(AERO_DOF_WHEEL);
  wheel_cur_vector_.resize(AERO_DOF_WHEEL);

  stroke_joint_indices_.clear();
  stroke_joint_indices_.reserve(AERO_DOF_LOWER);

  wheel_indices_.clear();
  wheel_indices_.reserve(AERO_DOF_WHEEL);

  // adding code
}

//////////////////////////////////////////////////
AeroLowerController::~AeroLowerController()
{
}

//////////////////////////////////////////////////
void AeroLowerController::servo_on()
{
  servo_command(1, 0);
}

//////////////////////////////////////////////////
void AeroLowerController::servo_off()
{
  servo_command(0, 0);
}

//////////////////////////////////////////////////
void AeroLowerController::wheel_on()
{
  servo_command(1, 1);
}

//////////////////////////////////////////////////
void AeroLowerController::servo_command(int16_t _d0, int16_t _d1)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<int16_t> stroke_vector(stroke_joint_indices_.size(), _d0);
  std::vector<uint8_t> dat(RAW_DATA_LENGTH);

  stroke_to_raw_(stroke_vector, dat);

  // adding code

  ser_.send_command(CMD_MOTOR_SRV, 0, dat);
}

//////////////////////////////////////////////////
void AeroLowerController::set_wheel_velocity(
    std::vector<int16_t>& _wheel_vector, uint16_t _time)
{
  boost::mutex::scoped_lock lock(ctrl_mtx_);

  std::vector<uint8_t> dat(RAW_DATA_LENGTH);

  // use previous reference strokes to keep its positions
  stroke_to_raw_(stroke_ref_vector_, dat);

  // wheel to raw
  for (size_t i = 0; i < wheel_indices_.size(); ++i)
  {
    AJointIndex& aji = wheel_indices_[i];
    encode_short_(_wheel_vector[aji.stroke_index],
                  &dat[RAW_HEADER_OFFSET + aji.raw_index * 2]);
  }
  ser_.send_command(CMD_MOVE_ABS, _time, dat);

  // MoveAbs returns current stroke
  std::vector<uint8_t> dummy;
  dummy.resize(RAW_DATA_LENGTH);
  ser_.read(dummy);
}

//////////////////////////////////////////////////
int32_t AeroLowerController::get_wheel_id(std::string& _name)
{
  for (size_t i = 0; i < wheel_indices_.size(); ++i)
  {
    if (wheel_indices_[i].joint_name == _name)
      return static_cast<int32_t>(wheel_indices_[i].stroke_index);
  }
  return -1;
}

//////////////////////////////////////////////////
std::string AeroLowerController::get_wheel_name(size_t _idx)
{
  return wheel_indices_[_idx].joint_name;
}

//////////////////////////////////////////////////
std::vector<int16_t>& AeroLowerController::get_reference_wheel_vector()
{
  return wheel_ref_vector_;
}
