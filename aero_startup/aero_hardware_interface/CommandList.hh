#ifndef AERO_CONTROLLER_COMMAND_LIST_H_
#define AERO_CONTROLLER_COMMAND_LIST_H_

namespace aero
{
  namespace controller
  {
    // header offset = 5bytes
    const static size_t RAW_HEADER_OFFSET = 5;
    // data length = 68bytes
    const static size_t RAW_DATA_LENGTH = 68;

    // command list
    const static uint8_t CMD_MOTOR_CUR  = 0x01; // CMAX
    const static uint8_t CMD_MOTOR_SPD  = 0x02; // SMAX
    const static uint8_t CMD_MOTOR_ACC  = 0x03; // ACCEL
    const static uint8_t CMD_MOTOR_GAIN = 0x04; // GAIN
    const static uint8_t CMD_MOTOR_PLS  = 0x05; // RATE
    const static uint8_t CMD_MOTOR_RET  = 0x06; // RETURN

    const static uint8_t CMD_MOVE_ABS_POS = 0x11; // TMOVE
    const static uint8_t CMD_MOVE_CUR_POS = 0x12; // CMOVE
    const static uint8_t CMD_MOVE_SPD_POS = 0x13; // SMOVE
    const static uint8_t CMD_MOVE_ABS_POS_RET = 0x14; // MOVE
    const static uint8_t CMD_MOVE_SPD = 0x15; // TURN //for wheels
    const static uint8_t CMD_MOVE_INC_POS_RET = 0x16; // INC

    const static uint8_t CMD_MOTOR_SRV = 0x21; // servo

    const static uint8_t CMD_GET_POS = 0x41;
    const static uint8_t CMD_GET_CUR = 0x42;
    const static uint8_t CMD_GET_TMP_VOLT = 0x43;
    const static uint8_t CMD_GET_AD  = 0x44;
    const static uint8_t CMD_GET_DIO = 0x45;

    const static uint8_t CMD_GET_VERSION = 0x51;
    const static uint8_t CMD_WATCH_MISSTEP = 0x52;
  }
}

#endif  // AERO_CONTROLLER_COMMAND_LIST_H_
