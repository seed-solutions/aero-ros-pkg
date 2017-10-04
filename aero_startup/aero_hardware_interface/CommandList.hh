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
    const static uint8_t CMD_MOTOR_CUR = 0x01;
    const static uint8_t CMD_MOTOR_ACC = 0x03;
    const static uint8_t CMD_MOTOR_GAIN = 0x04;

    const static uint8_t CMD_MOVE_ABS = 0x14;
    const static uint8_t CMD_MOVE_SPD = 0x15; // wheels

    const static uint8_t CMD_MOTOR_SRV = 0x21; // servo

    const static uint8_t CMD_GET_POS = 0x41;
    const static uint8_t CMD_GET_CUR = 0x42;
    const static uint8_t CMD_GET_TMP = 0x43;
    const static uint8_t CMD_GET_AD = 0x44;
    const static uint8_t CMD_GET_DIO = 0x45;

    const static uint8_t CMD_WATCH_MISSTEP = 0x52;
  }
}

#endif  // AERO_CONTROLLER_COMMAND_LIST_H_
